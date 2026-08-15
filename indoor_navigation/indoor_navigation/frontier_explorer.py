#!/usr/bin/env python3
from __future__ import annotations

import math
import os
from typing import Optional, Tuple, List

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import tf2_ros

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateToPose

from indoor_navigation.frontier_utils import occupancygrid_to_numpy, extract_frontiers, Frontier


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__("frontier_explorer")

        # ---- Params
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("min_cluster_size", 20)
        self.declare_parameter("publish_period_s", 1.0)

        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_link")

        self.declare_parameter("nav_action_name", "/navigate_to_pose")
        self.declare_parameter("min_goal_separation_m", 0.75)
        self.declare_parameter("goal_cooldown_s", 3.0)

        # frontier scoring (simple)
        self.declare_parameter("score_distance_weight", 1.0)
        self.declare_parameter("score_size_weight", 0.05)    

        self.map_topic = self.get_parameter("map_topic").value
        self.min_cluster_size = int(self.get_parameter("min_cluster_size").value)
        self.publish_period_s = float(self.get_parameter("publish_period_s").value)
        self.global_frame = self.get_parameter("global_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.nav_action_name = self.get_parameter("nav_action_name").value
        self.min_goal_separation_m = float(self.get_parameter("min_goal_separation_m").value)
        self.goal_cooldown_s = float(self.get_parameter("goal_cooldown_s").value)
        self.w_dist = float(self.get_parameter("score_distance_weight").value)
        self.w_size = float(self.get_parameter("score_size_weight").value)

        # ---- State
        self._map: Optional[OccupancyGrid] = None
        self._navigating: bool = False
        self._last_goal_xy: Optional[Tuple[float, float]] = None
        self._last_goal_time = self.get_clock().now()
        self._map_saved = False

        # ---- ROS I/O
        self._sub = self.create_subscription(OccupancyGrid, self.map_topic, self._on_map, 10)
        self._pub_markers = self.create_publisher(MarkerArray, "/frontiers", 10)

        # ---- TF
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ---- Nav2 action client
        self._nav_client = ActionClient(self, NavigateToPose, self.nav_action_name)

        self._timer = self.create_timer(self.publish_period_s, self._on_timer)
        self.get_logger().info(
            f"FrontierExplorer: map_topic={self.map_topic}, action={self.nav_action_name}, "
            f"global_frame={self.global_frame}, base_frame={self.base_frame}"
        )

        #“projection” du goal vers le libre + fallback BFS autour du centroïde
        self.declare_parameter("goal_backoff_m", 0.6)          # recul vers l'intérieur
        self.declare_parameter("goal_search_radius_m", 1.5)    # fallback autour du centroïde
        self.declare_parameter("goal_clearance_cells", 2)      # sécurité (en cellules)
        self.declare_parameter("save_map_on_completion", False)

        default_map_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "indoor_navigation", "map"))
        default_map_path = os.path.join(default_map_dir, "explored_map.pgm")
        self.declare_parameter("map_save_path", default_map_path)

        self.goal_backoff_m = float(self.get_parameter("goal_backoff_m").value)
        self.goal_search_radius_m = float(self.get_parameter("goal_search_radius_m").value)
        self.goal_clearance_cells = int(self.get_parameter("goal_clearance_cells").value)
        self.save_map_on_completion = bool(self.get_parameter("save_map_on_completion").value)
        self.map_save_path = self.get_parameter("map_save_path").value

    def _on_map(self, msg: OccupancyGrid):
        self._map = msg
        self._map_saved = False

    def _get_robot_xy(self) -> Optional[Tuple[float, float]]:
        # Try base_frame then base_footprint as fallback
        for bf in (self.base_frame, "base_footprint"):
            try:
                t = self._tf_buffer.lookup_transform(self.global_frame, bf, rclpy.time.Time())
                return float(t.transform.translation.x), float(t.transform.translation.y)
            except Exception:
                continue
        self.get_logger().warn("TF lookup failed for robot pose (map->base_link/base_footprint).")
        return None

    def _score_frontier(self, f: Frontier, robot_xy: Tuple[float, float]) -> float:
        rx, ry = robot_xy
        fx, fy = f.centroid
        dist = math.hypot(fx - rx, fy - ry)
        
        return (self.w_size * float(f.size)) - (self.w_dist * dist)

    def _pick_best_frontier(self, frontiers: List[Frontier], robot_xy: Tuple[float, float]) -> Optional[Frontier]:
        if not frontiers:
            return None
        best = None
        best_score = -1e18
        for f in frontiers:
            s = self._score_frontier(f, robot_xy)
            if s > best_score:
                best_score = s
                best = f
        return best

    def _goal_too_close_to_last(self, gx: float, gy: float) -> bool:
        if self._last_goal_xy is None:
            return False
        lx, ly = self._last_goal_xy
        return (gx - lx) ** 2 + (gy - ly) ** 2 < (self.min_goal_separation_m ** 2)

    def _cooldown_ok(self) -> bool:
        dt = (self.get_clock().now() - self._last_goal_time).nanoseconds * 1e-9
        return dt >= self.goal_cooldown_s

    def _send_nav_goal(self, gx: float, gy: float, robot_xy: Tuple[float, float]):
        if not self._nav_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn("NavigateToPose action server not available yet.")
            return

        rx, ry = robot_xy
        yaw = math.atan2(gy - ry, gx - rx)
        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)

        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = self.global_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(gx)
        ps.pose.position.y = float(gy)
        ps.pose.position.z = 0.0
        ps.pose.orientation.z = float(qz)
        ps.pose.orientation.w = float(qw)
        goal.pose = ps

        self.get_logger().info(f"Sending Nav2 goal to ({gx:.2f}, {gy:.2f}) yaw={yaw:.2f} rad")

        self._navigating = True
        self._last_goal_xy = (gx, gy)
        self._last_goal_time = self.get_clock().now()

        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().warn("Nav2 goal rejected.")
            self._navigating = False
            return

        self.get_logger().info("Nav2 goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        try:
            status = future.result().status
            self.get_logger().info(f"Nav2 goal finished with status={status}")
        except Exception as e:
            self.get_logger().warn(f"Failed reading Nav2 result: {e}")
        self._navigating = False

    def _write_map_pgm(self, m: OccupancyGrid) -> bool:
        try:
            directory = os.path.dirname(self.map_save_path)
            if directory and not os.path.exists(directory):
                os.makedirs(directory, exist_ok=True)

            width = int(m.info.width)
            height = int(m.info.height)
            header = f"P5\n{width} {height}\n255\n"
            data = bytearray(width * height)

            for idx, value in enumerate(m.data):
                if value == -1:
                    data[idx] = 205
                elif value >= 65:
                    data[idx] = 0
                else:
                    data[idx] = 254

            with open(self.map_save_path, "wb") as out_file:
                out_file.write(header.encode("ascii"))
                out_file.write(data)

            self.get_logger().info(f"Map saved to {self.map_save_path}")
            return True
        except Exception as exc:
            self.get_logger().warn(f"Failed to save map PGM: {exc}")
            return False

    def _publish_markers(self, m: OccupancyGrid, frontiers: List[Frontier], chosen: Optional[Frontier]):
        ma = MarkerArray()

        # Clear all previous
        clear = Marker()
        clear.header.frame_id = m.header.frame_id or self.global_frame
        clear.header.stamp = self.get_clock().now().to_msg()
        clear.ns = "frontiers"
        clear.id = 0
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        # Frontier centroids
        for i, f in enumerate(frontiers, start=1):
            mk = Marker()
            mk.header.frame_id = clear.header.frame_id
            mk.header.stamp = clear.header.stamp
            mk.ns = "frontiers"
            mk.id = i
            mk.type = Marker.SPHERE
            mk.action = Marker.ADD
            mk.pose.position.x = float(f.centroid[0])
            mk.pose.position.y = float(f.centroid[1])
            mk.pose.position.z = 0.1
            mk.pose.orientation.w = 1.0
            mk.scale.x = 0.20
            mk.scale.y = 0.20
            mk.scale.z = 0.20
            mk.color.r = 0.2
            mk.color.g = 0.8
            mk.color.b = 1.0
            mk.color.a = min(1.0, 0.25 + 0.02 * f.size)
            ma.markers.append(mk)

        # Chosen frontier marker (bigger sphere)
        if chosen is not None:
            mk = Marker()
            mk.header.frame_id = clear.header.frame_id
            mk.header.stamp = clear.header.stamp
            mk.ns = "frontier_goal"
            mk.id = 1
            mk.type = Marker.SPHERE
            mk.action = Marker.ADD
            mk.pose.position.x = float(chosen.centroid[0])
            mk.pose.position.y = float(chosen.centroid[1])
            mk.pose.position.z = 0.15
            mk.pose.orientation.w = 1.0
            mk.scale.x = 0.35
            mk.scale.y = 0.35
            mk.scale.z = 0.35
            mk.color.r = 1.0
            mk.color.g = 0.6
            mk.color.b = 0.1
            mk.color.a = 1.0
            ma.markers.append(mk)

        self._pub_markers.publish(ma)

    
    def _world_to_grid(self, wx: float, wy: float, ox: float, oy: float, res: float) -> tuple[int, int]:
        gx = int((wx - ox) / res)
        gy = int((wy - oy) / res)
        return gx, gy

    def _grid_in_bounds(self, gx: int, gy: int, w: int, h: int) -> bool:
        return 0 <= gx < w and 0 <= gy < h

    def _is_free(self, grid, gx: int, gy: int) -> bool:
        # free=0, unknown=-1, occupied>=50/100 (selon map)
        v = int(grid[gy, gx])
        return v == 0

    def _free_with_clearance(self, grid, gx: int, gy: int, w: int, h: int, r: int) -> bool:
        # exige un petit voisinage libre autour (évite goal collé au mur/inconnu)
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                nx, ny = gx + dx, gy + dy
                if not self._grid_in_bounds(nx, ny, w, h):
                    return False
                if int(grid[ny, nx]) != 0:
                    return False
        return True

    def _project_goal_into_free(
        self,
        grid,
        w: int, h: int,
        ox: float, oy: float, res: float,
        robot_xy: tuple[float, float],
        frontier_xy: tuple[float, float],
    ) -> tuple[float, float] | None:
        rx, ry = robot_xy
        fx, fy = frontier_xy

        # direction frontier -> robot (on recule vers le robot = vers du connu)
        vx = rx - fx
        vy = ry - fy
        norm = math.hypot(vx, vy)
        if norm < 1e-6:
            return None
        vx /= norm
        vy /= norm

        # essaie plusieurs reculs (du plus grand au plus petit)
        for backoff in [self.goal_backoff_m, 0.5, 0.4, 0.3, 0.2]:
            gxw = fx + vx * backoff
            gyw = fy + vy * backoff
            gx, gy = self._world_to_grid(gxw, gyw, ox, oy, res)
            if not self._grid_in_bounds(gx, gy, w, h):
                continue
            if self._free_with_clearance(grid, gx, gy, w, h, self.goal_clearance_cells):
                return gxw, gyw

        return None

    def _search_nearby_free(
        self,
        grid,
        w: int, h: int,
        ox: float, oy: float, res: float,
        center_xy: tuple[float, float],
    ) -> tuple[float, float] | None:
        # recherche locale carré autour du centroïde
        cx, cy = center_xy
        cgx, cgy = self._world_to_grid(cx, cy, ox, oy, res)
        radius_cells = int(self.goal_search_radius_m / res)

        best = None
        best_d2 = 1e18
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                gx, gy = cgx + dx, cgy + dy
                if not self._grid_in_bounds(gx, gy, w, h):
                    continue
                if not self._free_with_clearance(grid, gx, gy, w, h, self.goal_clearance_cells):
                    continue
                d2 = dx*dx + dy*dy
                if d2 < best_d2:
                    best_d2 = d2
                    wx = ox + (gx + 0.5) * res
                    wy = oy + (gy + 0.5) * res
                    best = (wx, wy)

        return best

    def _on_timer(self):
        if self._map is None:
            return

        m = self._map
        w = int(m.info.width)
        h = int(m.info.height)
        res = float(m.info.resolution)
        ox = float(m.info.origin.position.x)
        oy = float(m.info.origin.position.y)

        grid = occupancygrid_to_numpy(m.data, w, h)
        frontiers = extract_frontiers(
            grid, origin_xy=(ox, oy), resolution=res, min_cluster_size=self.min_cluster_size
        )

        if not frontiers and self.save_map_on_completion and not self._map_saved and not self._navigating:
            if self._write_map_pgm(m):
                self._map_saved = True

        robot_xy = self._get_robot_xy()
        chosen = None
        if robot_xy is not None and frontiers:
            chosen = self._pick_best_frontier(frontiers, robot_xy)

        # Always publish markers (debug)
        self._publish_markers(m, frontiers, chosen)

        # Decide if we send a goal
        if chosen is None or robot_xy is None:
            self.get_logger().info(f"Frontiers: {len(frontiers)} (no goal)")
            return

        self.get_logger().info(
            f"Frontiers: {len(frontiers)} | navigating={self._navigating}"
        )

        if self._navigating:
            return

        fx, fy = chosen.centroid

        # 1) projection vers l'intérieur du libre
        projected = self._project_goal_into_free(
            grid, w, h, ox, oy, res,
            robot_xy=robot_xy,
            frontier_xy=(fx, fy),
        )

        # 2) fallback: chercher un point libre proche
        if projected is None:
            projected = self._search_nearby_free(grid, w, h, ox, oy, res, (fx, fy))

        if projected is None:
            self.get_logger().warn("No valid free goal found near frontier (projection + search failed).")
            return

        gx, gy = projected

        if not self._cooldown_ok():
            return

        if self._goal_too_close_to_last(gx, gy):
            return

        self._send_nav_goal(gx, gy, robot_xy)


def main():
    rclpy.init()
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()