from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Sequence

# ─────────────────────────────────────────────────────────────────────────────
# Fonctions utilitaires (identiques à path_controller.py)
# ─────────────────────────────────────────────────────────────────────────────

def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _distance(ax: float, ay: float, bx: float, by: float) -> float:
    return math.hypot(bx - ax, by - ay)


# ─────────────────────────────────────────────────────────────────────────────
# Dataclasses (même format que path_controller.py pour compatibilité directe)
# ─────────────────────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Twist2D:
    linear: float
    angular: float


@dataclass(frozen=True)
class TrajectoryPoint:
    x: float
    y: float
    yaw: float | None = None
    speed: float | None = None


class ControlStatus(str, Enum):
    NO_PLAN      = "NO_PLAN"
    RUNNING      = "RUNNING"
    GOAL_REACHED = "GOAL_REACHED"
    STUCK        = "STUCK"


@dataclass(frozen=True)
class ControlCommand:
    twist:            Twist2D
    status:           ControlStatus
    target_index:     int
    distance_to_goal: float


# ─────────────────────────────────────────────────────────────────────────────
# Configuration Stanley
# Les paramètres communs avec PathController gardent les mêmes valeurs par
# défaut pour faciliter la comparaison des deux controllers.
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class StanleyConfig:
    # Gain Stanley : sensibilité au cross-track error
    # Plus k est grand, plus le robot corrige agressivement l'erreur latérale
    k: float = 0.75

    # Wheel base réel extrait de robot_core_04.xacro
    # wheel_offset_y = 0.1485 m → wheel_base = 2 * 0.1485 = 0.297 m
    wheel_base: float = 0.297

    # Limites de vitesse (identiques à ControllerConfig pour cohérence)
    max_linear_speed:  float = 0.5
    max_angular_speed: float = 1.0
    max_linear_accel:  float = 0.33
    max_angular_accel: float = 3.2

    # Vitesse minimale pour éviter la division par zéro dans atan(k*cte/v)
    min_tracking_speed: float = 0.05

    # Tolérance pour déclarer le goal atteint
    goal_xy_tolerance:  float = 0.15
    goal_yaw_tolerance: float = 0.25

    # Détection de blocage (STUCK) : même logique que PathController
    progress_min_distance: float = 0.08
    progress_timeout_s:    float = 3.0

    # Ralentissement à l'approche du goal
    goal_slowdown_radius: float = 0.8

    # Angle de braquage maximum (45 degrés)
    max_steering_angle: float = math.pi / 4


# ─────────────────────────────────────────────────────────────────────────────
# Contrôleur Stanley
# ─────────────────────────────────────────────────────────────────────────────

class StanleyController:
    """
    Contrôleur Stanley pour robot à roues différentielles.

    Interface identique à PathController pour une intégration directe
    dans l'environnement ROS 2 / Nav2 sans modification du reste du code.

    Algorithme Stanley :
        1. Trouver le waypoint le plus proche (fenêtre locale)
        2. Calculer heading_error (psi) et cross_track_error (cte)
        3. delta = psi + atan(k * cte / v)
        4. omega = v * tan(delta) / wheel_base
        5. Retourner Twist2D(linear=v, angular=omega)

    Référence : Stanley: The Robot that Won the DARPA Grand Challenge (Thrun et al.)
    """

    def __init__(self, config: StanleyConfig | None = None) -> None:
        self.config = config or StanleyConfig()
        self._path: list[TrajectoryPoint] = []
        self._target_index: int = 0
        self._last_twist = Twist2D(0.0, 0.0)
        self._progress_ref_pose: Pose2D | None = None
        self._time_since_progress: float = 0.0

    def reset(self) -> None:
        """Remet le contrôleur à zéro (appelé automatiquement par set_plan)."""
        self._target_index     = 0
        self._last_twist       = Twist2D(0.0, 0.0)
        self._progress_ref_pose = None
        self._time_since_progress = 0.0

    def set_plan(self, path: Sequence[TrajectoryPoint]) -> None:
        """
        Charge un nouveau chemin et remet le contrôleur à zéro.
        Appelé par Nav2 à chaque nouveau plan de navigation.
        """
        self._path = list(path)
        self.reset()

    @property
    def has_plan(self) -> bool:
        """Vrai si un chemin est chargé."""
        return len(self._path) > 0

    def is_goal_reached(self, pose: Pose2D) -> bool:
        """
        Vérifie si le robot est arrivé au dernier waypoint.
        Tient compte de la tolérance en position et en orientation.
        """
        if not self._path:
            return False

        goal = self._path[-1]
        dist = _distance(pose.x, pose.y, goal.x, goal.y)
        if dist > self.config.goal_xy_tolerance:
            return False

        # Si le dernier waypoint n'a pas d'orientation cible, on accepte
        if goal.yaw is None:
            return True

        yaw_err = abs(_normalize_angle(goal.yaw - pose.yaw))
        return yaw_err <= self.config.goal_yaw_tolerance

    def compute_command(
        self,
        current_pose: Pose2D,
        dt: float,
        current_twist: Twist2D | None = None,
    ) -> ControlCommand:
        """
        Calcule la commande de vitesse à appliquer au robot.

        Appelé à chaque cycle de contrôle par Nav2 ou le simulateur.

        Args:
            current_pose  : pose actuelle du robot (x, y, yaw)
            dt            : pas de temps en secondes (doit être > 0)
            current_twist : vitesse actuelle mesurée (optionnel)

        Returns:
            ControlCommand avec twist, status, target_index, distance_to_goal
        """
        if dt <= 0.0:
            raise ValueError("dt must be > 0")

        # Cas 1 : pas de plan chargé
        if not self._path:
            stop = Twist2D(0.0, 0.0)
            self._last_twist = stop
            return ControlCommand(
                twist=stop,
                status=ControlStatus.NO_PLAN,
                target_index=-1,
                distance_to_goal=math.inf,
            )

        # Cas 2 : goal atteint
        if self.is_goal_reached(current_pose):
            stop = Twist2D(0.0, 0.0)
            self._last_twist = stop
            return ControlCommand(
                twist=stop,
                status=ControlStatus.GOAL_REACHED,
                target_index=len(self._path) - 1,
                distance_to_goal=0.0,
            )

        # Cas 3 : détection de blocage (STUCK)
        status = self._update_progress(current_pose, dt)
        if status == ControlStatus.STUCK:
            stop = Twist2D(0.0, 0.0)
            self._last_twist = stop
            return ControlCommand(
                twist=stop,
                status=status,
                target_index=self._target_index,
                distance_to_goal=self._distance_to_goal(current_pose),
            )

        # Cas 4 : calcul Stanley normal
        # Étape 1 : trouver le waypoint le plus proche (fenêtre locale)
        self._target_index = self._find_nearest_index(current_pose)

        # Étape 2 : calculer les erreurs Stanley
        psi = self._heading_error(current_pose, self._target_index)
        cte = self._cross_track_error(current_pose, self._target_index)

        # Étape 3 : vitesse adaptative selon CTE et distance au goal
        v = self._compute_desired_linear_speed(current_pose, cte)

        # Étape 4 : angle de braquage Stanley
        # delta = heading_error + atan(k * CTE / v)
        delta = psi + math.atan2(
            self.config.k * cte,
            max(v, self.config.min_tracking_speed)
        )
        delta = _clamp(delta, -self.config.max_steering_angle, self.config.max_steering_angle)

        # Étape 5 : conversion angle de braquage → yaw-rate
        # omega = v * tan(delta) / wheel_base
        # C'est la cinématique correcte pour un robot différentiel
        omega = v * math.tan(delta) / self.config.wheel_base

        # Étape 6 : appliquer les limites d'accélération (rampe)
        desired   = Twist2D(linear=v, angular=omega)
        commanded = self._apply_limits(
            desired=desired,
            current=(current_twist or self._last_twist),
            dt=dt,
        )
        self._last_twist = commanded

        return ControlCommand(
            twist=commanded,
            status=ControlStatus.RUNNING,
            target_index=self._target_index,
            distance_to_goal=self._distance_to_goal(current_pose),
        )

    # ─────────────────────────────────────────────────────────────────────────
    # Méthodes internes
    # ─────────────────────────────────────────────────────────────────────────

    def _find_nearest_index(self, pose: Pose2D) -> int:
        """
        Trouve le waypoint le plus proche dans une fenêtre locale.
        On part de target_index - 1 pour éviter de reculer sur la trajectoire.
        Fenêtre limitée à 50 points pour rester O(1) sur longues trajectoires.
        """
        start    = max(0, self._target_index - 1)
        end      = min(start + 50, len(self._path))
        best_idx = start
        best_dist = math.inf

        for i in range(start, end):
            p = self._path[i]
            d = _distance(pose.x, pose.y, p.x, p.y)
            if d < best_dist:
                best_dist = d
                best_idx  = i

        return best_idx

    def _heading_error(self, pose: Pose2D, index: int) -> float:
        """
        Erreur d'orientation entre le robot et la trajectoire locale.
        Normalisée dans [-pi, pi].
        """
        if index >= len(self._path) - 1:
            index = len(self._path) - 2

        dx         = self._path[index + 1].x - self._path[index].x
        dy         = self._path[index + 1].y - self._path[index].y
        path_angle = math.atan2(dy, dx)

        return _normalize_angle(path_angle - pose.yaw)

    def _cross_track_error(self, pose: Pose2D, index: int) -> float:
        """
        Erreur latérale signée entre le robot et le segment courant.
        Signe positif : robot à gauche de la trajectoire.
        Signe négatif : robot à droite de la trajectoire.
        """
        if index >= len(self._path) - 1:
            index = len(self._path) - 2

        ax, ay = self._path[index].x,     self._path[index].y
        bx, by = self._path[index + 1].x, self._path[index + 1].y
        rx, ry = pose.x, pose.y

        dx     = bx - ax
        dy     = by - ay
        length = math.hypot(dx, dy)

        if length < 1e-6:
            return 0.0

        # Produit vectoriel signé → distance latérale
        return (dx * (ay - ry) - dy * (ax - rx)) / length

    def _compute_desired_linear_speed(self, pose: Pose2D, cte: float) -> float:
        """
        Vitesse linéaire adaptative :
        - Ralentit proportionnellement au CTE (erreur latérale)
        - Ralentit à l'approche du goal
        - Toujours bornée entre min_tracking_speed et max_linear_speed
        """
        c    = self.config
        base = c.max_linear_speed

        # Ralentissement à l'approche du goal
        dist_goal = self._distance_to_goal(pose)
        if dist_goal < c.goal_slowdown_radius:
            ratio = _clamp(dist_goal / max(c.goal_slowdown_radius, 1e-6), 0.0, 1.0)
            base *= ratio

        # Ralentissement selon le CTE
        base = base - 0.5 * abs(cte)

        return _clamp(base, c.min_tracking_speed, c.max_linear_speed)

    def _apply_limits(self, desired: Twist2D, current: Twist2D, dt: float) -> Twist2D:
        """
        Applique les limites d'accélération pour éviter les sauts brusques
        de vitesse (rampe linéaire identique à PathController).
        """
        c = self.config

        target_linear  = _clamp(desired.linear,  -c.max_linear_speed,  c.max_linear_speed)
        target_angular = _clamp(desired.angular, -c.max_angular_speed, c.max_angular_speed)

        max_dv = c.max_linear_accel  * dt
        max_dw = c.max_angular_accel * dt

        linear  = current.linear  + _clamp(target_linear  - current.linear,  -max_dv, max_dv)
        angular = current.angular + _clamp(target_angular - current.angular, -max_dw, max_dw)

        linear  = _clamp(linear,  -c.max_linear_speed,  c.max_linear_speed)
        angular = _clamp(angular, -c.max_angular_speed, c.max_angular_speed)

        return Twist2D(linear=linear, angular=angular)

    def _distance_to_goal(self, pose: Pose2D) -> float:
        """Distance euclidienne entre le robot et le dernier waypoint."""
        goal = self._path[-1]
        return _distance(pose.x, pose.y, goal.x, goal.y)

    def _update_progress(self, pose: Pose2D, dt: float) -> ControlStatus:
        """
        Détecte si le robot est bloqué (STUCK).
        Si le robot n'a pas avancé de progress_min_distance en
        progress_timeout_s secondes, on retourne STUCK.
        Logique identique à PathController.
        """
        if self._progress_ref_pose is None:
            self._progress_ref_pose   = pose
            self._time_since_progress = 0.0
            return ControlStatus.RUNNING

        moved = _distance(
            self._progress_ref_pose.x,
            self._progress_ref_pose.y,
            pose.x,
            pose.y,
        )

        if moved >= self.config.progress_min_distance:
            self._progress_ref_pose   = pose
            self._time_since_progress = 0.0
            return ControlStatus.RUNNING

        self._time_since_progress += dt
        if self._time_since_progress > self.config.progress_timeout_s:
            return ControlStatus.STUCK

        return ControlStatus.RUNNING

    def add_waypoint(self, point: TrajectoryPoint) -> None:
        """
        Ajoute un waypoint à la fin du chemin sans reset le controller.
        Contrairement à set_plan(), _target_index est préservé →
        le robot continue là où il en est sans repartir du début.
        """
        self._path.append(point)

    def remove_reached_waypoints(self, pose: Pose2D) -> None:
        """
        Supprime les waypoints déjà atteints par le robot.
        Appelé à chaque cycle pour garder la liste propre.
        """
        tol = self.config.goal_xy_tolerance
        while self._path and self._target_index > 0:
            wp = self._path[0]
            if _distance(pose.x, pose.y, wp.x, wp.y) < tol:
                self._path.pop(0)
                self._target_index = max(0, self._target_index - 1)
            else:
                break
