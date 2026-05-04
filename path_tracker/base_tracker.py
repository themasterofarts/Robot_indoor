from abc import ABC, abstractmethod

class BaseTracker(ABC):

    @abstractmethod
    def compute_velocities(self, robot_state, trajectory, current_index):
        raise NotImplementedError
