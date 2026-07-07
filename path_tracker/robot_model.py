import math
from .base_tracker import Pose2D, Twist2D


# Vitesse angulaire maximale autorisée (rad/s)
MAX_OMEGA = 2.0

# Wheel base réel du robot extrait de robot_core_04.xacro
# wheel_offset_y = 0.1485 m → wheel_base = 2 * 0.1485 = 0.297 m
WHEEL_BASE = 0.297


class DifferentialRobot:
    """
    Modèle cinématique d'un robot à roues différentielles.

    Le robot est décrit par sa pose (x, y, yaw) et se déplace
    selon les commandes Twist2D(linear, angular) reçues du tracker.

    Dimensions réelles extraites de robot_core_04.xacro :
        wheel_radius   = 0.06 m
        wheel_base     = 0.297 m (2 * wheel_offset_y)

    Cinématique utilisée (modèle unicycle) :
        x_new   = x + v * cos(yaw) * dt
        y_new   = y + v * sin(yaw) * dt
        yaw_new = yaw + omega * dt
    """

    def __init__(self, wheel_base: float = WHEEL_BASE):
        """
        Args:
            wheel_base : distance entre les deux roues (défaut = 0.297 m depuis URDF)
        """
        self.wheel_base = wheel_base

    def update(self, pose: Pose2D, twist: Twist2D, dt: float) -> Pose2D:
        """
        Applique la commande de vitesse et retourne la nouvelle pose du robot.

        CORRECTION : on reçoit maintenant Twist2D(linear, angular) au lieu
        de v_left/v_right. C'est la convention du repo (path_controller.py).

        Args:
            pose  : pose actuelle du robot (x, y, yaw)
            twist : commande de vitesse (linear m/s, angular rad/s)
            dt    : pas de temps en secondes

        Returns:
            nouvelle pose du robot après déplacement
        """
        v = twist.linear

        # Saturation de omega pour éviter des rotations irréalistes
        omega = max(-MAX_OMEGA, min(MAX_OMEGA, twist.angular))

        # Intégration Euler du modèle unicycle
        new_x   = pose.x   + v * math.cos(pose.yaw) * dt
        new_y   = pose.y   + v * math.sin(pose.yaw) * dt
        new_yaw = pose.yaw + omega * dt

        # Normalisation de l'angle pour rester dans [-pi, pi]
        new_yaw = math.atan2(math.sin(new_yaw), math.cos(new_yaw))

        return Pose2D(x=new_x, y=new_y, yaw=new_yaw)
