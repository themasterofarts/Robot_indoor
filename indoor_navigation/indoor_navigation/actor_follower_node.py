#!/usr/bin/env python3
"""
actor_follower_node.py
----------------------
ROS2 node that orchestrates actor following.

Pipeline (50 Hz loop — non-bloquant) :
  /actor/pose  ──►  compute_follow_goal_predictive()  ──►  _request_replan()
  /map         ──►  theta_star_with_fallback() [ProcessPoolExecutor]
  /tf          ──►  controller.compute_command()  ──►  /cmd_vel

Changelog complet :
  Fix 1    : Prédiction vitesse acteur (compute_follow_goal_predictive).
  Fix 2a   : Fallback euclidien rayon 15 cellules (_find_nearest_free_goal).
  Fix 2b   : Compteur d'échecs unifié _fail_count (STUCK + no-path).
  Fix 3    : ProcessPoolExecutor — Theta* hors GIL, timer 50 Hz non-bloquant.
  Fix A    : Reset contrôleur + stop immédiat sur no-path (plus de collision
             sur plan obsolète).
  Fix B    : theta_star_with_fallback — inflation adaptative couloirs étroits.
  ControllerConfig : paramètres vitesse corrigés (lenteur sur ligne droite).
  Codex 1  : _current_goal stocke le goal STATIQUE pour que should_replan()
             compare des valeurs cohérentes — évite replan à chaque cycle.
  Codex 2  : world_to_grid() utilise math.floor() (dans follower_functions).
  Codex 3  : Topic /actor/pose configurable via paramètre ROS2.
"""

from __future__ import annotations

import math
from concurrent.futures import ProcessPoolExecutor, Future
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid
import tf2_ros

from indoor_navigation.follower_functions import (
    MapInfo,
    WayPoint,
    quaternion_to_yaw,
    compute_distance,
    compute_follow_goal,
    compute_follow_goal_predictive,
    should_replan,
    theta_star_with_fallback,
)
from indoor_navigation.path_controller import (
    PathController,
    ControllerConfig,
    ControlStatus,
    Pose2D,
    TrajectoryPoint,
)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
CONTROL_HZ         = 50
FOLLOW_DISTANCE    = 0.5
REPLAN_THRESHOLD   = 0.3
ROBOT_RADIUS       = 0.20
STOP_DISTANCE      = 0.45

# Fix 1 — prédiction
PREDICTION_HORIZON = 0.4
MAX_ACTOR_SPEED    = 1.5

# Fix 2b — échecs unifiés
FAIL_MAX           = 3
FOLLOW_DIST_MAX    = 1.5
FOLLOW_DIST_STEP   = 0.25
FOLLOW_DIST_DECAY  = 0.02

# Codex 3 — topic par défaut (peut être surchargé via paramètre)
DEFAULT_ACTOR_TOPIC = "/actor/pose"

MAP_FRAME  = "map"
BASE_FRAME = "base_link"


# ---------------------------------------------------------------------------
# Fonction top-level pour ProcessPoolExecutor
# Doit être importable par les workers — hors de toute classe.
# ---------------------------------------------------------------------------

def _run_theta_star(
    start_world: tuple[float, float],
    goal_world: tuple[float, float],
    grid_data: list[int],
    info: MapInfo,
    robot_radius_m: float,
) -> list[WayPoint]:
    """Wrapper top-level appelé dans le process worker (Fix 3 + Fix B)."""
    return theta_star_with_fallback(
        start_world, goal_world, grid_data, info, robot_radius_m
    )


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class ActorFollowerNode(Node):

    def __init__(self) -> None:
        super().__init__("actor_follower_node")

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter("follow_distance", FOLLOW_DISTANCE)
        self.declare_parameter("replan_threshold", REPLAN_THRESHOLD)
        self.declare_parameter("robot_radius", ROBOT_RADIUS)
        self.declare_parameter("control_hz", float(CONTROL_HZ))
        self.declare_parameter("prediction_horizon", PREDICTION_HORIZON)
        self.declare_parameter("max_actor_speed", MAX_ACTOR_SPEED)
        # Codex 3 — topic configurable
        self.declare_parameter("actor_pose_topic", DEFAULT_ACTOR_TOPIC)

        self._follow_dist         = self.get_parameter("follow_distance").value
        self._follow_dist_nominal = self._follow_dist
        self._replan_thr          = self.get_parameter("replan_threshold").value
        self._robot_radius        = self.get_parameter("robot_radius").value
        hz                        = self.get_parameter("control_hz").value
        self._prediction_horizon  = self.get_parameter("prediction_horizon").value
        self._max_actor_speed     = self.get_parameter("max_actor_speed").value
        actor_topic               = self.get_parameter("actor_pose_topic").value

        # ── State ────────────────────────────────────────────────────────────
        self._actor_pose: PoseStamped | None = None
        self._map: OccupancyGrid | None = None
        self._map_info: MapInfo | None = None

        # Codex 1 — _current_goal stocke le goal STATIQUE (compute_follow_goal)
        # pour que should_replan() compare des valeurs cohérentes.
        # Le goal prédictif est passé à Theta* mais n'est PAS stocké ici.
        self._current_goal: tuple[float, float] | None = None

        self._seen_actor_pose   = False
        self._seen_map          = False
        self._seen_tf           = False
        self._warned_actor_pose = False
        self._warned_map        = False
        self._warned_tf         = False

        # Fix 1 — état précédent acteur
        self._prev_actor_x: float = 0.0
        self._prev_actor_y: float = 0.0
        self._prev_actor_time: float = 0.0

        # Fix 2b — compteur échecs unifié
        self._fail_count: int = 0

        # Fix 3 — ProcessPoolExecutor
        self._executor_pool = ProcessPoolExecutor(max_workers=1)
        self._future: Future | None = None
        # _pending_goal stocke le goal STATIQUE associé au future en cours
        # (pour mise à jour de _current_goal après succès Theta*)
        self._pending_static_goal: tuple[float, float] | None = None

        # ── ControllerConfig corrigé (lenteur sur ligne droite) ──────────────
        cfg = ControllerConfig(
            max_linear_speed=0.8,
            max_angular_speed=1.5,
            max_linear_accel=1.0,
            max_angular_accel=3.2,
            min_lookahead=0.4,
            max_lookahead=1.5,
            lookahead_gain=1.5,
            curvature_speed_coeff=0.8,
            goal_slowdown_radius=0.3,
            min_tracking_speed=0.05,
            goal_xy_tolerance=0.15,
            goal_yaw_tolerance=0.25,
            progress_min_distance=0.08,
            progress_timeout_s=3.0,
        )
        self._controller = PathController(cfg)

        # ── TF ───────────────────────────────────────────────────────────────
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── Subscribers ──────────────────────────────────────────────────────
        # Codex 3 — topic configurable via paramètre
        self.create_subscription(
            PoseStamped, actor_topic, self._actor_pose_cb, 10
        )
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid, "/map", self._map_cb, map_qos
        )

        # ── Publisher ────────────────────────────────────────────────────────
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # ── Timer ────────────────────────────────────────────────────────────
        self._dt = 1.0 / hz
        self._prev_time = self.get_clock().now()
        self.create_timer(self._dt, self._control_loop)

        self.get_logger().info(
            f"ActorFollowerNode ready — "
            f"actor_topic={actor_topic}, "
            f"follow_distance={self._follow_dist} m, "
            f"loop={hz} Hz, "
            f"prediction_horizon={self._prediction_horizon} s"
        )

    # ────────────────────────────────────────────────────────────────────────
    # Callbacks
    # ────────────────────────────────────────────────────────────────────────

    def _actor_pose_cb(self, msg: PoseStamped) -> None:
        self._actor_pose = msg
        if not self._seen_actor_pose:
            self.get_logger().info("Received first actor pose message")
            self._seen_actor_pose = True
        self._warned_actor_pose = False

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self._map = msg
        meta = msg.info
        self._map_info = MapInfo(
            resolution=meta.resolution,
            width=meta.width,
            height=meta.height,
            origin_x=meta.origin.position.x,
            origin_y=meta.origin.position.y,
        )
        if not self._seen_map:
            self.get_logger().info(
                f"Received /map: {meta.width}x{meta.height} "
                f"@ {meta.resolution:.3f} m/cell"
            )
            self._seen_map = True
        self._warned_map = False

    # ────────────────────────────────────────────────────────────────────────
    # TF
    # ────────────────────────────────────────────────────────────────────────

    def _get_robot_pose(self) -> Pose2D | None:
        try:
            tf = self._tf_buffer.lookup_transform(
                MAP_FRAME, BASE_FRAME,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
            if not self._seen_tf:
                self.get_logger().info("Received TF map -> base_link")
                self._seen_tf = True
            self._warned_tf = False
            t = tf.transform.translation
            r = tf.transform.rotation
            return Pose2D(
                x=t.x, y=t.y,
                yaw=quaternion_to_yaw(r.x, r.y, r.z, r.w)
            )
        except Exception:
            return None

    # ────────────────────────────────────────────────────────────────────────
    # Control loop — 50 Hz, non-bloquant (Fix 3)
    # ────────────────────────────────────────────────────────────────────────

    def _control_loop(self) -> None:
        now = self.get_clock().now()
        dt = (now - self._prev_time).nanoseconds * 1e-9
        self._prev_time = now
        if dt <= 0.0:
            dt = self._dt

        # ── Guards ──────────────────────────────────────────────────────────
        if self._actor_pose is None:
            if not self._warned_actor_pose:
                self.get_logger().warn("Waiting for actor pose ...")
                self._warned_actor_pose = True
            self._stop()
            return

        if self._map is None or self._map_info is None:
            if not self._warned_map:
                self.get_logger().warn("Waiting for /map ...")
                self._warned_map = True
            self._stop()
            return

        robot = self._get_robot_pose()
        if robot is None:
            if not self._warned_tf:
                self.get_logger().warn("Waiting for TF map -> base_link ...")
                self._warned_tf = True
            self._stop()
            return

        # ── Pose acteur ──────────────────────────────────────────────────────
        ap = self._actor_pose.pose
        actor_yaw = quaternion_to_yaw(
            ap.orientation.x, ap.orientation.y,
            ap.orientation.z, ap.orientation.w,
        )
        actor_x, actor_y = ap.position.x, ap.position.y

        # Fix 1 — dt acteur pour estimation de vitesse
        now_sec = now.nanoseconds * 1e-9
        actor_dt = (
            now_sec - self._prev_actor_time
            if self._prev_actor_time > 1e-6 else self._dt
        )

        # ── Proximité ────────────────────────────────────────────────────────
        if compute_distance(robot.x, robot.y, actor_x, actor_y) <= STOP_DISTANCE:
            self._stop()
            self._controller.set_plan([])
            self._current_goal = None
            self._prev_actor_x = actor_x
            self._prev_actor_y = actor_y
            self._prev_actor_time = now_sec
            return

        # ── Fix 3 : consommer le résultat du process worker si prêt ─────────
        if self._future is not None and self._future.done():
            try:
                waypoints: list[WayPoint] = self._future.result()
                if waypoints:
                    traj = [
                        TrajectoryPoint(x=wp.x, y=wp.y, yaw=wp.yaw, speed=wp.speed)
                        for wp in waypoints
                    ]
                    self._controller.set_plan(traj)
                    # Codex 1 — on stocke le goal statique, pas le prédictif
                    self._current_goal = self._pending_static_goal
                    self._fail_count = 0
                    self.get_logger().info(
                        f"Plan appliqué : {len(traj)} waypoints, "
                        f"goal statique=({self._current_goal[0]:.2f},"
                        f"{self._current_goal[1]:.2f})"
                    )
                else:
                    # Fix A — reset immédiat sur no-path pour éviter
                    # que le robot continue sur un plan obsolète vers un mur
                    self.get_logger().warn(
                        f"Theta* : aucun chemin vers "
                        f"({self._pending_static_goal[0]:.2f},"
                        f"{self._pending_static_goal[1]:.2f})"
                    )
                    self._controller.set_plan([])
                    self._current_goal = None
                    self._stop()
                    self._handle_failure("no-path")

            except Exception as exc:
                self.get_logger().error(f"Erreur process Theta* : {exc}")
                self._controller.set_plan([])
                self._current_goal = None
                self._stop()
                self._handle_failure("exception")

            finally:
                self._future = None

        # ── Déclenchement replanning (non-bloquant) ──────────────────────────
        need_replan = (
            self._current_goal is None
            or should_replan(
                self._current_goal,
                actor_x, actor_y, actor_yaw,
                self._follow_dist,
                self._replan_thr,
            )
        )
        if need_replan:
            self._request_replan(
                robot,
                actor_x, actor_y, actor_yaw,
                self._prev_actor_x, self._prev_actor_y,
                actor_dt,
            )

        # Mémoriser la pose acteur
        self._prev_actor_x = actor_x
        self._prev_actor_y = actor_y
        self._prev_actor_time = now_sec

        # ── Pas de plan ──────────────────────────────────────────────────────
        if not self._controller.has_plan:
            self._stop("No plan available")
            return

        # ── Commande de contrôle (< 1 ms) ────────────────────────────────────
        cmd_result = self._controller.compute_command(robot, dt)
        status = cmd_result.status

        if status == ControlStatus.RUNNING:
            twist = Twist()
            twist.linear.x = cmd_result.twist.linear
            twist.angular.z = cmd_result.twist.angular
            self._cmd_vel_pub.publish(twist)
            # Fix 2b — retour progressif à follow_dist nominale
            self._fail_count = 0
            if self._follow_dist > self._follow_dist_nominal:
                self._follow_dist = max(
                    self._follow_dist - FOLLOW_DIST_DECAY,
                    self._follow_dist_nominal,
                )

        elif status == ControlStatus.GOAL_REACHED:
            self._stop()
            self._current_goal = None

        elif status == ControlStatus.STUCK:
            # Fix A + Fix 2b — invalidation totale du plan + compteur unifié
            self.get_logger().warn("Robot STUCK")
            self._controller.set_plan([])
            self._current_goal = None
            self._stop()
            self._handle_failure("stuck")

        elif status == ControlStatus.NO_PLAN:
            self._stop()

    # ────────────────────────────────────────────────────────────────────────
    # Fix 2b — gestion unifiée des échecs (STUCK + no-path)
    # ────────────────────────────────────────────────────────────────────────

    def _handle_failure(self, reason: str) -> None:
        """
        Incrémente _fail_count. Après FAIL_MAX échecs consécutifs
        (toutes causes : STUCK ou Theta* no-path), augmente follow_distance
        pour viser un point plus dégagé.
        """
        self._fail_count += 1
        self.get_logger().warn(
            f"Échec [{reason}] — {self._fail_count}/{FAIL_MAX}"
        )
        if self._fail_count >= FAIL_MAX:
            self._follow_dist = min(
                self._follow_dist + FOLLOW_DIST_STEP,
                FOLLOW_DIST_MAX,
            )
            self.get_logger().warn(
                f"Échecs répétés — follow_distance → {self._follow_dist:.2f} m"
            )
            self._fail_count = 0

    # ────────────────────────────────────────────────────────────────────────
    # Fix 3 — Theta* dans un process séparé (hors GIL)
    # ────────────────────────────────────────────────────────────────────────

    def _request_replan(
        self,
        robot: Pose2D,
        actor_x: float, actor_y: float, actor_yaw: float,
        prev_actor_x: float, prev_actor_y: float,
        actor_dt: float,
    ) -> None:
        """
        Soumet Theta* au ProcessPoolExecutor si aucun calcul n'est en cours.
        Le process worker reçoit des copies sérialisées — aucun état partagé.

        Codex 1 — on calcule et mémorise le goal STATIQUE comme référence
        pour should_replan(), et on passe le goal PRÉDICTIF à Theta* pour
        que le robot anticipe les virages.
        """
        if self._future is not None and not self._future.done():
            return

        # Goal statique — référence pour should_replan() et _current_goal
        static_gx, static_gy = compute_follow_goal(
            actor_x, actor_y, actor_yaw, self._follow_dist
        )

        # Goal prédictif — soumis à Theta* (Fix 1)
        pred_gx, pred_gy = compute_follow_goal_predictive(
            actor_x, actor_y, actor_yaw,
            prev_actor_x, prev_actor_y,
            actor_dt,
            follow_distance=self._follow_dist,
            prediction_horizon=self._prediction_horizon,
            max_actor_speed=self._max_actor_speed,
        )

        # Snapshots — données immuables copiées avant soumission au worker
        map_data = list(self._map.data)
        map_info = self._map_info
        robot_radius = self._robot_radius

        self.get_logger().debug(
            f"[process] Planning: robot=({robot.x:.2f},{robot.y:.2f}) "
            f"goal_pred=({pred_gx:.2f},{pred_gy:.2f}) "
            f"goal_static=({static_gx:.2f},{static_gy:.2f})"
        )

        # Mémoriser le goal statique — sera stocké dans _current_goal au retour
        self._pending_static_goal = (static_gx, static_gy)

        # Soumettre Theta* avec le goal prédictif
        self._future = self._executor_pool.submit(
            _run_theta_star,
            (robot.x, robot.y),
            (pred_gx, pred_gy),
            map_data,
            map_info,
            robot_radius,
        )

    # ────────────────────────────────────────────────────────────────────────
    # Helpers
    # ────────────────────────────────────────────────────────────────────────

    def _stop(self, reason: str = "") -> None:
        if reason:
            self.get_logger().debug(reason)
        self._cmd_vel_pub.publish(Twist())

    def destroy_node(self) -> None:
        self._executor_pool.shutdown(wait=False)
        super().destroy_node()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = ActorFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
