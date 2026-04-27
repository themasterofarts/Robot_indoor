#!/usr/bin/env python3
"""
actor_follower_node.py
----------------------
ROS2 node that orchestrates actor following.

Pipeline (50 Hz loop):
  /actor/pose  ──►  compute_follow_goal()
  /map         ──►  theta_star()  ──►  controller.set_plan()
  /tf (map→base_link) ──►  controller.compute_command()  ──►  /cmd_vel

External pure-Python modules (no ROS2):
  - follower_functions  : Theta* planner + geometry helpers
  - path_controller     : Regulated Pure Pursuit controller

Topics subscribed:
  /actor/pose   geometry_msgs/PoseStamped
  /map          nav_msgs/OccupancyGrid

Topics published:
  /cmd_vel      geometry_msgs/Twist

TF lookup:
  map  →  base_link   (robot pose in world frame)
"""

from __future__ import annotations

import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid
import tf2_ros

# Pure-Python modules (same package, no ROS deps)
from indoor_navigation.follower_functions import (
    MapInfo,
    WayPoint,
    quaternion_to_yaw,
    compute_distance,
    compute_follow_goal_predictive,
    should_replan,
    theta_star,
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
CONTROL_HZ        = 50          # Hz — control loop frequency
FOLLOW_DISTANCE   = 0.5         # metres — stay this far behind the actor
REPLAN_THRESHOLD  = 0.3         # metres — actor displacement that triggers replan
ROBOT_RADIUS      = 0.20        # metres — used for obstacle inflation in Theta*
STOP_DISTANCE     = 0.45        # metres — stop if already within this range

# Fix 1
PREDICTION_HORIZON = 0.4        # seconds — actor motion lookahead
MAX_ACTOR_SPEED    = 1.5        # m/s — clamp for noisy pose jumps

# Fix 2b
FAIL_MAX           = 3          # consecutive failures before increasing follow distance
FOLLOW_DIST_MAX    = 1.5        # metres — upper bound for adaptive follow distance
FOLLOW_DIST_STEP   = 0.25       # metres — increase after repeated failures
FOLLOW_DIST_DECAY  = 0.02       # metres — decay back to nominal on success

# TF frames
MAP_FRAME        = "map"
BASE_FRAME       = "base_link"


class ActorFollowerNode(Node):
    """ROS2 node: subscribes to actor pose + map, runs Theta* + RPP, drives robot."""

    def __init__(self) -> None:
        super().__init__("actor_follower_node")

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter("follow_distance", FOLLOW_DISTANCE)
        self.declare_parameter("replan_threshold", REPLAN_THRESHOLD)
        self.declare_parameter("robot_radius",    ROBOT_RADIUS)
        self.declare_parameter("control_hz",      float(CONTROL_HZ))
        self.declare_parameter("prediction_horizon", PREDICTION_HORIZON)
        self.declare_parameter("max_actor_speed", MAX_ACTOR_SPEED)

        self._follow_dist  = self.get_parameter("follow_distance").value
        self._follow_dist_nominal = self._follow_dist
        self._replan_thr   = self.get_parameter("replan_threshold").value
        self._robot_radius = self.get_parameter("robot_radius").value
        hz                 = self.get_parameter("control_hz").value
        self._prediction_horizon = self.get_parameter("prediction_horizon").value
        self._max_actor_speed = self.get_parameter("max_actor_speed").value

        # ── State ────────────────────────────────────────────────────────────
        self._actor_pose: PoseStamped | None = None
        self._map:        OccupancyGrid | None = None
        self._map_info:   MapInfo | None = None
        self._current_goal: tuple[float, float] | None = None  # (wx, wy)
        self._prev_actor_x: float = 0.0
        self._prev_actor_y: float = 0.0
        self._prev_actor_time: float = 0.0
        self._fail_count: int = 0
        self._seen_actor_pose = False
        self._seen_map = False
        self._seen_tf = False
        self._warned_actor_pose = False
        self._warned_map = False
        self._warned_tf = False

        # ── Controller ───────────────────────────────────────────────────────
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
        self.create_subscription(
            PoseStamped, "/actor/pose", self._actor_pose_cb, 10
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

        # ── Control loop timer ───────────────────────────────────────────────
        self._dt = 1.0 / hz
        self._prev_time = self.get_clock().now()
        self.create_timer(self._dt, self._control_loop)

        self.get_logger().info(
            f"ActorFollowerNode ready — follow_distance={self._follow_dist} m, "
            f"loop={hz} Hz"
        )

    # ────────────────────────────────────────────────────────────────────────
    # Callbacks
    # ────────────────────────────────────────────────────────────────────────

    def _actor_pose_cb(self, msg: PoseStamped) -> None:
        self._actor_pose = msg
        if not self._seen_actor_pose:
            self.get_logger().info("Received first /actor/pose message")
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
                f"Received /map: {meta.width}x{meta.height} @ {meta.resolution:.3f} m/cell"
            )
            self._seen_map = True
        self._warned_map = False

    # ────────────────────────────────────────────────────────────────────────
    # TF helpers
    # ────────────────────────────────────────────────────────────────────────

    def _get_robot_pose(self) -> Pose2D | None:
        """Lookup map → base_link transform and return as Pose2D."""
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
            yaw = quaternion_to_yaw(r.x, r.y, r.z, r.w)
            return Pose2D(x=t.x, y=t.y, yaw=yaw)
        except Exception:
            return None

    # ────────────────────────────────────────────────────────────────────────
    # Main control loop (50 Hz)
    # ────────────────────────────────────────────────────────────────────────

    def _control_loop(self) -> None:
        now = self.get_clock().now()
        dt  = (now - self._prev_time).nanoseconds * 1e-9
        self._prev_time = now
        if dt <= 0.0:
            dt = self._dt  # fallback on first tick

        # ── Guard: wait for all data ─────────────────────────────────────────
        if self._actor_pose is None:
            if not self._warned_actor_pose:
                self.get_logger().warn("Waiting for /actor/pose ...")
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

        # ── Extract actor pose ───────────────────────────────────────────────
        ap = self._actor_pose.pose
        actor_yaw = quaternion_to_yaw(
            ap.orientation.x,
            ap.orientation.y,
            ap.orientation.z,
            ap.orientation.w,
        )
        actor_x, actor_y = ap.position.x, ap.position.y

        # ── Check proximity: already close enough → stop ─────────────────────
        dist_to_actor = compute_distance(robot.x, robot.y, actor_x, actor_y)
        if dist_to_actor <= STOP_DISTANCE:
            self._stop()
            self._controller.reset()
            self._current_goal = None
            self._prev_actor_x = actor_x
            self._prev_actor_y = actor_y
            self._prev_actor_time = now.nanoseconds * 1e-9
            return

        now_sec = now.nanoseconds * 1e-9
        actor_dt = (
            now_sec - self._prev_actor_time
            if self._prev_actor_time > 1e-6
            else self._dt
        )

        # ── Replan if needed ─────────────────────────────────────────────────
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
            self._run_planner(
                robot,
                actor_x, actor_y, actor_yaw,
                self._prev_actor_x, self._prev_actor_y,
                actor_dt,
            )

        self._prev_actor_x = actor_x
        self._prev_actor_y = actor_y
        self._prev_actor_time = now_sec

        # ── No valid plan yet ────────────────────────────────────────────────
        if not self._controller.has_plan:
            self._stop("No plan available")
            return

        # ── Compute command ──────────────────────────────────────────────────
        cmd_result = self._controller.compute_command(robot, dt)

        status = cmd_result.status
        if status == ControlStatus.RUNNING:
            twist = Twist()
            twist.linear.x  = cmd_result.twist.linear
            twist.angular.z = cmd_result.twist.angular
            self._cmd_vel_pub.publish(twist)
            self._fail_count = 0
            if self._follow_dist > self._follow_dist_nominal:
                self._follow_dist = max(
                    self._follow_dist - FOLLOW_DIST_DECAY,
                    self._follow_dist_nominal,
                )

        elif status == ControlStatus.GOAL_REACHED:
            self._stop()
            self._current_goal = None   # will replan when actor moves again

        elif status == ControlStatus.STUCK:
            self.get_logger().warn("Robot STUCK — forcing replanning")
            self._controller.reset()
            self._current_goal = None   # trigger replan on next cycle
            self._stop()
            self._handle_failure("stuck")

        elif status == ControlStatus.NO_PLAN:
            self._stop()

    # ────────────────────────────────────────────────────────────────────────
    # Planner call
    # ────────────────────────────────────────────────────────────────────────

    def _run_planner(
        self,
        robot: Pose2D,
        actor_x: float, actor_y: float, actor_yaw: float,
        prev_actor_x: float, prev_actor_y: float,
        actor_dt: float,
    ) -> None:
        """Run Theta*, load result into controller, update current goal."""
        goal_x, goal_y = compute_follow_goal_predictive(
            actor_x, actor_y, actor_yaw,
            prev_actor_x, prev_actor_y,
            actor_dt,
            follow_distance=self._follow_dist,
            prediction_horizon=self._prediction_horizon,
            max_actor_speed=self._max_actor_speed,
        )

        self.get_logger().debug(
            f"Planning: robot=({robot.x:.2f},{robot.y:.2f}) "
            f"goal=({goal_x:.2f},{goal_y:.2f})"
        )

        waypoints: list[WayPoint] = theta_star(
            start_world=(robot.x, robot.y),
            goal_world=(goal_x, goal_y),
            grid_data=list(self._map.data),
            info=self._map_info,
            robot_radius_m=self._robot_radius,
        )

        if not waypoints:
            self.get_logger().warn(
                f"Theta* found no path to ({goal_x:.2f}, {goal_y:.2f})"
            )
            self._controller.reset()
            self._current_goal = None
            self._handle_failure("no-path")
            return

        # Convert WayPoint → TrajectoryPoint (structurally identical)
        traj = [
            TrajectoryPoint(x=wp.x, y=wp.y, yaw=wp.yaw, speed=wp.speed)
            for wp in waypoints
        ]
        self._controller.set_plan(traj)
        self._current_goal = (goal_x, goal_y)

        self.get_logger().info(
            f"New plan: {len(traj)} waypoints, "
            f"goal=({goal_x:.2f},{goal_y:.2f})"
        )

    def _handle_failure(self, reason: str) -> None:
        """Track consecutive failures and adapt the follow distance."""
        self._fail_count += 1
        self.get_logger().warn(
            f"Failure [{reason}] — {self._fail_count}/{FAIL_MAX}"
        )
        if self._fail_count >= FAIL_MAX:
            self._follow_dist = min(
                self._follow_dist + FOLLOW_DIST_STEP,
                FOLLOW_DIST_MAX,
            )
            self.get_logger().warn(
                f"Repeated failures — follow_distance -> {self._follow_dist:.2f} m"
            )
            self._fail_count = 0

    # ────────────────────────────────────────────────────────────────────────
    # Helpers
    # ────────────────────────────────────────────────────────────────────────

    def _stop(self, reason: str = "") -> None:
        """Publish zero-velocity command."""
        if reason:
            self.get_logger().debug(reason)
        self._cmd_vel_pub.publish(Twist())


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
