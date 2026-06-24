"""
Package path_tracker — Simulation de suivi de trajectoire
avec le contrôleur Stanley pour robot différentiel.

Dimensions du robot (robot_core_04.xacro) :
    wheel_base   = 0.297 m
    wheel_radius = 0.06 m
"""
from .base_tracker        import BaseTracker, Pose2D, Twist2D
from .robot_model         import DifferentialRobot
from .stanley             import StanleyTracker
from .stanley_controller  import StanleyController, StanleyConfig
from .trajectory          import combined_path
