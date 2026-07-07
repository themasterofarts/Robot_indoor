#!/usr/bin/env python3
"""
Nœud ROS 2 Stanley Controller — Suivi d'acteur par fil d'Ariane.

Utilise add_waypoint() pour ajouter les waypoints sans reset le controller.
Le robot continue là où il en est sans repartir du début à chaque nouveau waypoint.
"""
from __future__ import annotations

import math
from collections import deque

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry

from indoor_navigation.stanley_controller import (
    StanleyController,
    StanleyConfig,
    Pose2D,
    Twist2D,
    TrajectoryPoint,
    ControlStatus,
)


def _quaternion_to_yaw(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


class StanleyNode(Node):

    MIN_WAYPOINT_DIST = 0.25  # distance minimale entre deux waypoints (m)

    def __init__(self):
        super().__init__('stanley_node')

        self.declare_parameter('k',                  0.75)
        self.declare_parameter('wheel_base',         0.297)
        self.declare_parameter('max_linear_speed',   0.8)
        self.declare_parameter('max_angular_speed',  1.5)
        self.declare_parameter('goal_xy_tolerance',  0.20)
        self.declare_parameter('control_frequency',  20.0)
        self.declare_parameter('safety_distance',    0.6)
        self.declare_parameter('progress_timeout_s', 10.0)

        k                  = self.get_parameter('k').value
        wheel_base         = self.get_parameter('wheel_base').value
        max_linear_speed   = self.get_parameter('max_linear_speed').value
        max_angular_speed  = self.get_parameter('max_angular_speed').value
        goal_xy_tolerance  = self.get_parameter('goal_xy_tolerance').value
        control_frequency  = self.get_parameter('control_frequency').value
        self._safety_dist  = self.get_parameter('safety_distance').value
        progress_timeout_s = self.get_parameter('progress_timeout_s').value
        self._dt           = 1.0 / control_frequency

        config = StanleyConfig(
            k                  = k,
            wheel_base         = wheel_base,
            max_linear_speed   = max_linear_speed,
            max_angular_speed  = max_angular_speed,
            goal_xy_tolerance  = goal_xy_tolerance,
            progress_timeout_s = progress_timeout_s,
        )
        self._controller    = StanleyController(config=config)
        self._last_crumb:   Pose2D | None = None
        self._current_pose: Pose2D | None = None
        self._actor_pose:   Pose2D | None = None
        self._current_twist = Twist2D(0.0, 0.0)
        self._log_counter   = 0

        self.create_subscription(PoseStamped, '/actor/pose', self._on_actor_pose, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(self._dt, self._control_loop)

        self.get_logger().info(
            f'StanleyNode | k={k} | v_max={max_linear_speed} | '
            f'waypoint_dist={self.MIN_WAYPOINT_DIST}m | safety={self._safety_dist}m'
        )

    def _on_actor_pose(self, msg: PoseStamped) -> None:
        """
        Enregistre un waypoint si l'acteur a bougé de MIN_WAYPOINT_DIST.
        Utilise add_waypoint() pour ne pas reset le controller.
        """
        ax = msg.pose.position.x
        ay = msg.pose.position.y

        # Filtre distance : pivot sans déplacement = pas de waypoint
        if self._last_crumb is not None:
            dist = math.hypot(ax - self._last_crumb.x, ay - self._last_crumb.y)
            if dist < self.MIN_WAYPOINT_DIST:
                return

        actor_yaw = _quaternion_to_yaw(
            msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w,
        )
        self._actor_pose = Pose2D(x=ax, y=ay, yaw=actor_yaw)
        new_wp = TrajectoryPoint(x=ax, y=ay, yaw=actor_yaw)

        # add_waypoint() ajoute sans reset _target_index
        self._controller.add_waypoint(new_wp)
        self._last_crumb = Pose2D(x=ax, y=ay, yaw=actor_yaw)

    def _on_odom(self, msg: Odometry) -> None:
        yaw = _quaternion_to_yaw(
            msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w,
        )
        self._current_pose  = Pose2D(
            x=msg.pose.pose.position.x, y=msg.pose.pose.position.y, yaw=yaw
        )
        self._current_twist = Twist2D(
            linear=msg.twist.twist.linear.x, angular=msg.twist.twist.angular.z
        )

    def _control_loop(self) -> None:
        if self._current_pose is None:
            return

        # Supprimer les waypoints atteints sans reset
        self._controller.remove_reached_waypoints(self._current_pose)

        # Pas de waypoints → stop
        if not self._controller.has_plan:
            self._pub.publish(Twist())
            return

        # Distance de sécurité
        if self._actor_pose is not None:
            dist_to_actor = math.hypot(
                self._actor_pose.x - self._current_pose.x,
                self._actor_pose.y - self._current_pose.y,
            )
            if dist_to_actor <= self._safety_dist:
                self._pub.publish(Twist())
                return

        # Commande Stanley
        cmd = self._controller.compute_command(
            current_pose  = self._current_pose,
            dt            = self._dt,
            current_twist = self._current_twist,
        )

        if cmd.status == ControlStatus.STUCK:
            self.get_logger().warn('⚠️ STUCK — reset')
            self._controller.reset()

        # Log toutes les secondes
        self._log_counter += 1
        if self._log_counter >= int(1.0 / self._dt):
            self._log_counter = 0
            dist_actor = math.hypot(
                self._actor_pose.x - self._current_pose.x,
                self._actor_pose.y - self._current_pose.y,
            ) if self._actor_pose else -1
            self.get_logger().info(
                f'waypoints={len(self._controller._path)} | '
                f'target={self._controller._target_index} | '
                f'dist={dist_actor:.2f}m | '
                f'status={cmd.status.value} | '
                f'v={cmd.twist.linear:.2f} w={cmd.twist.angular:.2f}'
            )

        twist_msg           = Twist()
        twist_msg.linear.x  = cmd.twist.linear
        twist_msg.angular.z = cmd.twist.angular
        self._pub.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = StanleyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
