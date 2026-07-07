from abc import ABC, abstractmethod
from dataclasses import dataclass


@dataclass(frozen=True)
class Pose2D:
    """Position et orientation du robot dans le plan."""
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Twist2D:
    """Commande de vitesse : linéaire (m/s) et angulaire (rad/s)."""
    linear: float
    angular: float


class BaseTracker(ABC):
    """
    Classe abstraite commune à tous les controllers de trajectoire.
    Tout tracker (Stanley, Pure Pursuit, etc.) doit hériter de cette classe
    et implémenter compute_command().
    """

    @abstractmethod
    def compute_command(self, pose: Pose2D, trajectory: list, current_index: int, dt: float) -> tuple:
        """
        Calcule la commande de vitesse pour suivre la trajectoire.

        Args:
            pose          : état actuel du robot (x, y, yaw)
            trajectory    : liste de waypoints (x, y)
            current_index : index du waypoint courant
            dt            : pas de temps en secondes

        Returns:
            twist         : Twist2D(linear, angular)
            target_index  : index du waypoint ciblé par le tracker
        """
        raise NotImplementedError
