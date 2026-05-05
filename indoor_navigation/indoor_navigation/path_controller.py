# path_ontroller.py

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Sequence


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _distance(ax: float, ay: float, bx: float, by: float) -> float:
    return math.hypot(bx - ax, by - ay)


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
    NO_PLAN = "NO_PLAN"
    RUNNING = "RUNNING"
    GOAL_REACHED = "GOAL_REACHED"
    STUCK = "STUCK"


@dataclass(frozen=True)
class ControlCommand:
    twist: Twist2D
    status: ControlStatus
    target_index: int
    distance_to_goal: float


@dataclass(frozen=True)
class ControllerConfig:
    max_linear_speed: float = 0.5
    max_angular_speed: float = 1.0
    max_linear_accel: float = 0.33
    max_angular_accel: float = 3.2

    min_lookahead: float = 0.3
    max_lookahead: float = 1.2
    lookahead_gain: float = 1.2

    curvature_speed_coeff: float = 1.2
    goal_slowdown_radius: float = 0.8
    min_tracking_speed: float = 0.05

    goal_xy_tolerance: float = 0.15
    goal_yaw_tolerance: float = 0.25

    progress_min_distance: float = 0.08
    progress_timeout_s: float = 3.0


class PathController:
    """
    ROS2-independent trajectory controller for differential-drive robots.

    The design intentionally mimics Nav2 controller behavior:
    - set a path
    - compute v / w commands at each control cycle
    - report status (running, goal reached, stuck)
    """

    def __init__(self, config: ControllerConfig | None = None) -> None:
        self.config = config or ControllerConfig()
        self._path: list[TrajectoryPoint] = []
        self._target_index: int = 0
        self._last_twist = Twist2D(0.0, 0.0)
        self._progress_ref_pose: Pose2D | None = None
        self._time_since_progress: float = 0.0

    def reset(self) -> None:
        self._target_index = 0
        self._last_twist = Twist2D(0.0, 0.0)
        self._progress_ref_pose = None
        self._time_since_progress = 0.0

    def set_plan(self, path: Sequence[TrajectoryPoint]) -> None:
        self._path = list(path)
        self.reset()

    @property
    def has_plan(self) -> bool:
        return len(self._path) > 0

    def is_goal_reached(self, pose: Pose2D) -> bool:
        if not self._path:
            return False

        goal = self._path[-1]
        dist = _distance(pose.x, pose.y, goal.x, goal.y)
        if dist > self.config.goal_xy_tolerance:
            return False

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
        if dt <= 0.0:
            raise ValueError("dt must be > 0")

        if not self._path:
            stop = Twist2D(0.0, 0.0)
            self._last_twist = stop
            return ControlCommand(
                twist=stop,
                status=ControlStatus.NO_PLAN,
                target_index=-1,
                distance_to_goal=math.inf,
            )

        if self.is_goal_reached(current_pose):
            stop = Twist2D(0.0, 0.0)
            self._last_twist = stop
            return ControlCommand(
                twist=stop,
                status=ControlStatus.GOAL_REACHED,
                target_index=len(self._path) - 1,
                distance_to_goal=0.0,
            )

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

        nearest_idx = self._find_nearest_index(current_pose)
        self._target_index = self._find_lookahead_index(
            start_idx=nearest_idx,
            lookahead_distance=self._compute_lookahead(
                (current_twist.linear if current_twist else self._last_twist.linear)
            ),
        )
        target = self._path[self._target_index]

        local_x, local_y, look_dist = self._to_robot_frame(current_pose, target)
        if look_dist < 1e-6:
            desired_linear = 0.0
            desired_angular = 0.0
        else:
            curvature = 2.0 * local_y / max(look_dist * look_dist, 1e-6)
            desired_linear = self._compute_desired_linear_speed(current_pose, curvature)
            desired_angular = desired_linear * curvature

        commanded = self._apply_limits(
            desired=Twist2D(desired_linear, desired_angular),
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

    def _find_nearest_index(self, pose: Pose2D) -> int:
        start = max(0, self._target_index - 1)
        best_idx = start
        best_dist = math.inf

        for i in range(start, len(self._path)):
            p = self._path[i]
            d = _distance(pose.x, pose.y, p.x, p.y)
            if d < best_dist:
                best_dist = d
                best_idx = i
        return best_idx

    def _compute_lookahead(self, current_linear_speed: float) -> float:
        c = self.config
        lookahead = c.min_lookahead + c.lookahead_gain * abs(current_linear_speed)
        return _clamp(lookahead, c.min_lookahead, c.max_lookahead)

    def _find_lookahead_index(self, start_idx: int, lookahead_distance: float) -> int:
        if start_idx >= len(self._path) - 1:
            return len(self._path) - 1

        acc = 0.0
        prev = self._path[start_idx]
        for i in range(start_idx + 1, len(self._path)):
            nxt = self._path[i]
            acc += _distance(prev.x, prev.y, nxt.x, nxt.y)
            if acc >= lookahead_distance:
                return i
            prev = nxt
        return len(self._path) - 1

    def _to_robot_frame(self, pose: Pose2D, point: TrajectoryPoint) -> tuple[float, float, float]:
        dx = point.x - pose.x
        dy = point.y - pose.y
        cos_y = math.cos(pose.yaw)
        sin_y = math.sin(pose.yaw)

        local_x = cos_y * dx + sin_y * dy
        local_y = -sin_y * dx + cos_y * dy
        look_dist = math.hypot(local_x, local_y)
        return local_x, local_y, look_dist

    def _compute_desired_linear_speed(self, pose: Pose2D, curvature: float) -> float:
        c = self.config
        base = c.max_linear_speed
        dist_goal = self._distance_to_goal(pose)

        if dist_goal < c.goal_slowdown_radius:
            ratio = _clamp(dist_goal / max(c.goal_slowdown_radius, 1e-6), 0.0, 1.0)
            base *= ratio

        base = max(base, c.min_tracking_speed)
        curve_scale = 1.0 / (1.0 + c.curvature_speed_coeff * abs(curvature))
        return base * curve_scale

    def _apply_limits(self, desired: Twist2D, current: Twist2D, dt: float) -> Twist2D:
        c = self.config

        target_linear = _clamp(desired.linear, -c.max_linear_speed, c.max_linear_speed)
        target_angular = _clamp(desired.angular, -c.max_angular_speed, c.max_angular_speed)

        max_dv = c.max_linear_accel * dt
        max_dw = c.max_angular_accel * dt

        linear = current.linear + _clamp(target_linear - current.linear, -max_dv, max_dv)
        angular = current.angular + _clamp(target_angular - current.angular, -max_dw, max_dw)

        linear = _clamp(linear, -c.max_linear_speed, c.max_linear_speed)
        angular = _clamp(angular, -c.max_angular_speed, c.max_angular_speed)
        return Twist2D(linear=linear, angular=angular)

    def _distance_to_goal(self, pose: Pose2D) -> float:
        goal = self._path[-1]
        return _distance(pose.x, pose.y, goal.x, goal.y)

    def _update_progress(self, pose: Pose2D, dt: float) -> ControlStatus:
        if self._progress_ref_pose is None:
            self._progress_ref_pose = pose
            self._time_since_progress = 0.0
            return ControlStatus.RUNNING

        moved = _distance(
            self._progress_ref_pose.x,
            self._progress_ref_pose.y,
            pose.x,
            pose.y,
        )
        if moved >= self.config.progress_min_distance:
            self._progress_ref_pose = pose
            self._time_since_progress = 0.0
            return ControlStatus.RUNNING

        self._time_since_progress += dt
        if self._time_since_progress > self.config.progress_timeout_s:
            return ControlStatus.STUCK
        return ControlStatus.RUNNING
