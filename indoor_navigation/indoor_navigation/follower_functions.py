"""
follower_functions.py
---------------------
Pure Python (zero ROS2 / zero external deps) utilities for the actor follower.

Sections:
  1. Geometry helpers
  2. Occupancy-grid helpers
  3. Theta* path planner
  4. Follow-goal & replan helpers

All poses are represented as plain (x, y, yaw) values or as instances of the
path_controller.Pose2D / TrajectoryPoint dataclasses — the caller decides which
to use.  Functions that feed directly into PathController return TrajectoryPoint
objects so they are ready to pass to controller.set_plan().
"""

from __future__ import annotations

import math
import heapq
from dataclasses import dataclass
from typing import List, Optional, Tuple

# ---------------------------------------------------------------------------
# Type aliases (lightweight, no ROS dependency)
# ---------------------------------------------------------------------------

@dataclass
class MapInfo:
    """Mirrors the relevant fields of nav_msgs/MapMetaData."""
    resolution: float        # metres per cell
    width: int               # columns
    height: int              # rows
    origin_x: float          # world X of cell (0, 0)
    origin_y: float          # world Y of cell (0, 0)


# We re-export TrajectoryPoint-compatible namedtuple so callers can use this
# file without importing path_controller directly.  When the node imports both,
# it should use path_controller.TrajectoryPoint — they are structurally identical.
@dataclass(frozen=True)
class WayPoint:
    """Minimal waypoint compatible with path_controller.TrajectoryPoint."""
    x: float
    y: float
    yaw: Optional[float] = None
    speed: Optional[float] = None


# ---------------------------------------------------------------------------
# 1. Geometry helpers
# ---------------------------------------------------------------------------

def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Convert a quaternion to a yaw angle (radians, range [-π, π])."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    """Wrap an angle to [-π, π]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def compute_distance(x1: float, y1: float, x2: float, y2: float) -> float:
    """Euclidean distance between two 2-D points."""
    return math.hypot(x2 - x1, y2 - y1)


def compute_angle(from_x: float, from_y: float, to_x: float, to_y: float) -> float:
    """Heading angle (radians) from point A to point B."""
    return math.atan2(to_y - from_y, to_x - from_x)


def compute_yaw_along_path(waypoints: List[WayPoint]) -> List[WayPoint]:
    """
    Fill in the yaw field of each waypoint so it faces the next one.
    The last waypoint keeps the same yaw as the second-to-last.
    """
    if not waypoints:
        return waypoints

    result: List[WayPoint] = []
    for i, wp in enumerate(waypoints):
        if i < len(waypoints) - 1:
            nxt = waypoints[i + 1]
            yaw = compute_angle(wp.x, wp.y, nxt.x, nxt.y)
        else:
            yaw = result[-1].yaw if result else 0.0
        result.append(WayPoint(x=wp.x, y=wp.y, yaw=yaw, speed=wp.speed))
    return result


# ---------------------------------------------------------------------------
# 2. Occupancy-grid helpers
# ---------------------------------------------------------------------------

# Cells with occupancy value >= this threshold are considered occupied.
OCCUPANCY_THRESHOLD = 50


def world_to_grid(wx: float, wy: float, info: MapInfo) -> Tuple[int, int]:
    """
    Convert world coordinates (metres) to grid indices (col, row).
    Returns (-1, -1) if the point is outside the map.
    """
    col = int((wx - info.origin_x) / info.resolution)
    row = int((wy - info.origin_y) / info.resolution)
    if 0 <= col < info.width and 0 <= row < info.height:
        return col, row
    return -1, -1


def grid_to_world(col: int, row: int, info: MapInfo) -> Tuple[float, float]:
    """Convert grid indices (col, row) to world coordinates (centre of cell)."""
    wx = info.origin_x + (col + 0.5) * info.resolution
    wy = info.origin_y + (row + 0.5) * info.resolution
    return wx, wy


def is_cell_free(col: int, row: int,
                 grid_data: List[int], info: MapInfo) -> bool:
    """
    Return True if the cell is within bounds and not occupied.
    grid_data is the flat row-major OccupancyGrid.data array.
    Unknown cells (-1) are treated as free for path planning.
    """
    if col < 0 or row < 0 or col >= info.width or row >= info.height:
        return False
    value = grid_data[row * info.width + col]
    return value < OCCUPANCY_THRESHOLD  # -1 (unknown) also passes


def is_cell_free_inflated(col: int, row: int,
                          grid_data: List[int], info: MapInfo,
                          robot_radius_m: float = 0.20) -> bool:
    """
    Check a cell AND its neighbourhood (inflated by robot_radius_m).
    Avoids placing waypoints too close to walls.
    """
    radius_cells = int(math.ceil(robot_radius_m / info.resolution))
    for dc in range(-radius_cells, radius_cells + 1):
        for dr in range(-radius_cells, radius_cells + 1):
            if not is_cell_free(col + dc, row + dr, grid_data, info):
                return False
    return True


def line_of_sight(ax: int, ay: int, bx: int, by: int,
                  grid_data: List[int], info: MapInfo,
                  robot_radius_m: float = 0.20) -> bool:
    """
    Bresenham ray-cast from grid cell (ax, ay) to (bx, by).
    Returns True only if every cell along the segment is free
    (with inflation for the robot radius).
    """
    dx = abs(bx - ax)
    dy = abs(by - ay)
    x, y = ax, ay
    sx = 1 if bx > ax else -1
    sy = 1 if by > ay else -1
    err = dx - dy

    while True:
        if not is_cell_free_inflated(x, y, grid_data, info, robot_radius_m):
            return False
        if x == bx and y == by:
            return True
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy


# ---------------------------------------------------------------------------
# 3. Theta* path planner
# ---------------------------------------------------------------------------

def _heuristic(ax: int, ay: int, bx: int, by: int) -> float:
    """Octile distance heuristic — admissible for 8-connected grids."""
    dx = abs(bx - ax)
    dy = abs(by - ay)
    return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)


def _find_nearest_free_goal(
    gx: int, gy: int,
    grid_data: List[int], info: MapInfo,
    robot_radius_m: float,
    max_radius: int = 15,
) -> Tuple[int, int]:
    """
    Search the nearest free cell around a goal using concentric square borders.
    Returns (-1, -1) if no free cell is found within max_radius.
    """
    for r in range(1, max_radius + 1):
        candidates: List[Tuple[float, int, int]] = []
        for dc in range(-r, r + 1):
            for dr in range(-r, r + 1):
                if abs(dc) != r and abs(dr) != r:
                    continue
                ngx, ngy = gx + dc, gy + dr
                if is_cell_free_inflated(ngx, ngy, grid_data, info, robot_radius_m):
                    candidates.append((math.hypot(dc, dr), ngx, ngy))
        if candidates:
            candidates.sort()
            return candidates[0][1], candidates[0][2]
    return -1, -1


def theta_star(start_world: Tuple[float, float],
               goal_world: Tuple[float, float],
               grid_data: List[int],
               info: MapInfo,
               robot_radius_m: float = 0.20) -> List[WayPoint]:
    """
    Theta* any-angle path planner on a 2-D occupancy grid.

    Parameters
    ----------
    start_world : (wx, wy) robot position in world frame
    goal_world  : (wx, wy) goal position in world frame
    grid_data   : flat row-major OccupancyGrid.data  (int8 list)
    info        : MapInfo instance
    robot_radius_m : inflation radius for obstacle check

    Returns
    -------
    List of WayPoint with yaw filled in, ready for PathController.set_plan().
    Returns an empty list if no path is found.
    """
    sx, sy = world_to_grid(*start_world, info)
    gx, gy = world_to_grid(*goal_world, info)

    # Clamp goal if slightly outside map
    gx = max(0, min(info.width - 1, gx))
    gy = max(0, min(info.height - 1, gy))

    if sx == -1 or sy == -1:
        return []

    if not is_cell_free_inflated(gx, gy, grid_data, info, robot_radius_m):
        gx, gy = _find_nearest_free_goal(gx, gy, grid_data, info, robot_radius_m)
        if gx == -1:
            return []

    # g_score: cost to reach each cell
    g: dict[Tuple[int, int], float] = {(sx, sy): 0.0}
    # parent dict
    parent: dict[Tuple[int, int], Tuple[int, int]] = {(sx, sy): (sx, sy)}

    # open heap: (f, col, row)
    open_heap: list[Tuple[float, int, int]] = []
    heapq.heappush(open_heap, (_heuristic(sx, sy, gx, gy), sx, sy))
    closed: set[Tuple[int, int]] = set()

    # 8-connected neighbours
    neighbours = [(-1, -1), (-1, 0), (-1, 1),
                  ( 0, -1),          ( 0, 1),
                  ( 1, -1), ( 1, 0), ( 1, 1)]

    while open_heap:
        _, cx, cy = heapq.heappop(open_heap)
        node = (cx, cy)

        if node in closed:
            continue
        closed.add(node)

        if cx == gx and cy == gy:
            # Reconstruct path
            path_cells: List[Tuple[int, int]] = []
            current = (gx, gy)
            while current != parent[current]:
                path_cells.append(current)
                current = parent[current]
            path_cells.append((sx, sy))
            path_cells.reverse()

            # Convert to world WayPoints
            raw: List[WayPoint] = []
            for col, row in path_cells:
                wx, wy = grid_to_world(col, row, info)
                raw.append(WayPoint(x=wx, y=wy))

            return compute_yaw_along_path(raw)

        for dx, dy in neighbours:
            nx, ny = cx + dx, cy + dy
            neighbour = (nx, ny)

            if neighbour in closed:
                continue
            if not is_cell_free_inflated(nx, ny, grid_data, info, robot_radius_m):
                continue

            # Theta*: try line-of-sight from grandparent
            p = parent[node]
            px, py = p
            if line_of_sight(px, py, nx, ny, grid_data, info, robot_radius_m):
                # Path 2: go through grandparent directly
                move_cost = compute_distance(px, py, nx, ny) * info.resolution
                tentative_g = g[p] + move_cost
                if tentative_g < g.get(neighbour, math.inf):
                    g[neighbour] = tentative_g
                    parent[neighbour] = p
                    f = tentative_g + _heuristic(nx, ny, gx, gy)
                    heapq.heappush(open_heap, (f, nx, ny))
            else:
                # Path 1: standard A* step
                move_cost = math.sqrt(dx * dx + dy * dy) * info.resolution
                tentative_g = g[node] + move_cost
                if tentative_g < g.get(neighbour, math.inf):
                    g[neighbour] = tentative_g
                    parent[neighbour] = node
                    f = tentative_g + _heuristic(nx, ny, gx, gy)
                    heapq.heappush(open_heap, (f, nx, ny))

    # No path found
    return []


# ---------------------------------------------------------------------------
# 4. Follow-goal & replan helpers
# ---------------------------------------------------------------------------

def compute_follow_goal(actor_x: float, actor_y: float, actor_yaw: float,
                        follow_distance: float = 0.5) -> Tuple[float, float]:
    """
    Compute the point that is `follow_distance` metres *behind* the actor
    (i.e. opposite to the direction the actor is facing).

    Returns (goal_x, goal_y) in world frame.
    """
    goal_x = actor_x - follow_distance * math.cos(actor_yaw)
    goal_y = actor_y - follow_distance * math.sin(actor_yaw)
    return goal_x, goal_y


def compute_follow_goal_predictive(
    actor_x: float, actor_y: float, actor_yaw: float,
    prev_actor_x: float, prev_actor_y: float,
    dt: float,
    follow_distance: float = 0.5,
    prediction_horizon: float = 0.4,
    max_actor_speed: float = 1.5,
) -> Tuple[float, float]:
    """
    Estimate the actor future position by extrapolating its velocity,
    then compute a follow-goal behind that predicted position.
    """
    if dt > 1e-6:
        vx = (actor_x - prev_actor_x) / dt
        vy = (actor_y - prev_actor_y) / dt
        speed = math.hypot(vx, vy)
        if speed > max_actor_speed:
            scale = max_actor_speed / speed
            vx *= scale
            vy *= scale
    else:
        vx, vy = 0.0, 0.0

    predicted_x = actor_x + vx * prediction_horizon
    predicted_y = actor_y + vy * prediction_horizon

    goal_x = predicted_x - follow_distance * math.cos(actor_yaw)
    goal_y = predicted_y - follow_distance * math.sin(actor_yaw)
    return goal_x, goal_y


def should_replan(current_goal: Tuple[float, float],
                  actor_x: float, actor_y: float, actor_yaw: float,
                  follow_distance: float = 0.5,
                  move_threshold: float = 0.3) -> bool:
    """
    Return True if the actor has moved enough that the follow-goal has shifted
    by more than `move_threshold` metres — triggering a new Theta* call.

    Parameters
    ----------
    current_goal   : (x, y) of the goal used for the current plan
    actor_x/y/yaw  : current actor pose
    follow_distance: same value used in compute_follow_goal
    move_threshold : minimum displacement (m) to trigger replanning
    """
    new_goal = compute_follow_goal(actor_x, actor_y, actor_yaw, follow_distance)
    dist = compute_distance(current_goal[0], current_goal[1],
                            new_goal[0],     new_goal[1])
    return dist > move_threshold
