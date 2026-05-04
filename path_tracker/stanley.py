import math
from base_tracker import BaseTracker

class StanleyTracker(BaseTracker):

    def __init__(self, k=0.75, v_base=0.15, v_max=0.5, wheel_base=0.3):
        self.k          = k
        self.v_base     = v_base
        self.v_max      = v_max
        self.wheel_base = wheel_base

    def _find_target(self, robot_state, trajectory, current_index):
        min_dist   = float('inf')
        best_index = current_index

        for i in range(current_index, len(trajectory)):
            dx   = trajectory[i][0] - robot_state['x']
            dy   = trajectory[i][1] - robot_state['y']
            dist = math.sqrt(dx**2 + dy**2)
            if dist < min_dist:
                min_dist   = dist
                best_index = i

        return best_index, min_dist

    def _heading_error(self, robot_state, trajectory, index):
        if index >= len(trajectory) - 1:
            index = len(trajectory) - 2

        dx         = trajectory[index + 1][0] - trajectory[index][0]
        dy         = trajectory[index + 1][1] - trajectory[index][1]
        path_angle = math.atan2(dy, dx)
        error      = path_angle - robot_state['theta']
        error      = math.atan2(math.sin(error), math.cos(error))
        return error

    def _cross_track_error(self, robot_state, trajectory, index):
        if index >= len(trajectory) - 1:
            index = len(trajectory) - 2

        ax, ay = trajectory[index]
        bx, by = trajectory[index + 1]
        rx, ry = robot_state['x'], robot_state['y']

        dx = bx - ax
        dy = by - ay
        length = math.sqrt(dx**2 + dy**2)
        if length < 1e-6:
            return 0.0
        cte = (dx * (ay - ry) - dy * (ax - rx)) / length
        return cte

    def _adaptive_velocity(self, cte):
        v = self.v_base - 0.5 * abs(cte)
        return max(0.1, min(self.v_max, v))

    def compute_velocities(self, robot_state, trajectory, current_index):
        target_index, _ = self._find_target(
            robot_state, trajectory, current_index
        )

        psi   = self._heading_error(robot_state, trajectory, target_index)
        cte   = self._cross_track_error(robot_state, trajectory, target_index)
        v     = self._adaptive_velocity(cte)
        delta = psi + math.atan2(self.k * cte, v)
        delta = max(-math.pi / 4, min(math.pi / 4, delta))

        v_right = v + (delta * self.wheel_base / 2)
        v_left  = v - (delta * self.wheel_base / 2)

        return v_left, v_right
