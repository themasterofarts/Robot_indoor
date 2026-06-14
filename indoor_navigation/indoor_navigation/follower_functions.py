#follower_functions.py vf 


from __future__ import annotations

import math
import heapq
from dataclasses import dataclass
from typing import List, Optional, Tuple


@dataclass
class MapInfo:
    resolution: float
    width: int
    height: int
    origin_x: float
    origin_y: float


@dataclass(frozen=True)
class WayPoint:
    x: float
    y: float
    yaw: Optional[float] = None
    speed: Optional[float] = None


# ---------------------------------------------------------------------------
# 1. Geometry helpers
# ---------------------------------------------------------------------------

def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def compute_distance(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.hypot(x2 - x1, y2 - y1)


def compute_angle(from_x: float, from_y: float, to_x: float, to_y: float) -> float:
    return math.atan2(to_y - from_y, to_x - from_x)


def compute_yaw_along_path(waypoints: List[WayPoint]) -> List[WayPoint]:
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

OCCUPANCY_THRESHOLD = 50


def world_to_grid(wx: float, wy: float, info: MapInfo) -> Tuple[int, int]:
    "
    col = math.floor((wx - info.origin_x) / info.resolution)
    row = math.floor((wy - info.origin_y) / info.resolution)
    if 0 <= col < info.width and 0 <= row < info.height:
        return col, row
    return -1, -1


def grid_to_world(col: int, row: int, info: MapInfo) -> Tuple[float, float]:
    wx = info.origin_x + (col + 0.5) * info.resolution
    wy = info.origin_y + (row + 0.5) * info.resolution
    return wx, wy


def is_cell_free(col: int, row: int,
                 grid_data: List[int], info: MapInfo) -> bool:
    if col < 0 or row < 0 or col >= info.width or row >= info.height:
        return False
    return grid_data[row * info.width + col] < OCCUPANCY_THRESHOLD


def is_cell_free_inflated(col: int, row: int,
                          grid_data: List[int], info: MapInfo,
                          robot_radius_m: float = 0.20) -> bool:
    radius_cells = int(math.ceil(robot_radius_m / info.resolution))
    for dc in range(-radius_cells, radius_cells + 1):
        for dr in range(-radius_cells, radius_cells + 1):
            if not is_cell_free(col + dc, row + dr, grid_data, info):
                return False
    return True


def line_of_sight(ax: int, ay: int, bx: int, by: int,
                  grid_data: List[int], info: MapInfo,
                  robot_radius_m: float = 0.20) -> bool:
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
    dx = abs(bx - ax)
    dy = abs(by - ay)
    return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)


def _find_nearest_free_goal(
    gx: int, gy: int,
    grid_data: List[int], info: MapInfo,
    robot_radius_m: float,
    max_radius: int = 15,
) -> Tuple[int, int]:
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
    
    sx, sy = world_to_grid(*start_world, info)
    gx, gy = world_to_grid(*goal_world, info)

    gx = max(0, min(info.width - 1, gx))
    gy = max(0, min(info.height - 1, gy))

    if sx == -1 or sy == -1:
        return []

    if not is_cell_free_inflated(gx, gy, grid_data, info, robot_radius_m):
        result = _find_nearest_free_goal(gx, gy, grid_data, info, robot_radius_m)
        if result == (-1, -1):
            return []
        gx, gy = result

    g: dict[Tuple[int, int], float] = {(sx, sy): 0.0}
    parent: dict[Tuple[int, int], Tuple[int, int]] = {(sx, sy): (sx, sy)}
    open_heap: list[Tuple[float, int, int]] = []
    heapq.heappush(open_heap, (_heuristic(sx, sy, gx, gy), sx, sy))
    closed: set[Tuple[int, int]] = set()

    neighbours = [(-1, -1), (-1, 0), (-1, 1),
                  (0, -1),           (0, 1),
                  (1, -1),  (1, 0),  (1, 1)]

    while open_heap:
        _, cx, cy = heapq.heappop(open_heap)
        node = (cx, cy)
        if node in closed:
            continue
        closed.add(node)

        if cx == gx and cy == gy:
            path_cells: List[Tuple[int, int]] = []
            current = (gx, gy)
            while current != parent[current]:
                path_cells.append(current)
                current = parent[current]
            path_cells.append((sx, sy))
            path_cells.reverse()
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
            p = parent[node]
            px, py = p
            if line_of_sight(px, py, nx, ny, grid_data, info, robot_radius_m):
                move_cost = compute_distance(px, py, nx, ny) * info.resolution
                tentative_g = g[p] + move_cost
                if tentative_g < g.get(neighbour, math.inf):
                    g[neighbour] = tentative_g
                    parent[neighbour] = p
                    f = tentative_g + _heuristic(nx, ny, gx, gy)
                    heapq.heappush(open_heap, (f, nx, ny))
            else:
                move_cost = math.sqrt(dx * dx + dy * dy) * info.resolution
                tentative_g = g[node] + move_cost
                if tentative_g < g.get(neighbour, math.inf):
                    g[neighbour] = tentative_g
                    parent[neighbour] = node
                    f = tentative_g + _heuristic(nx, ny, gx, gy)
                    heapq.heappush(open_heap, (f, nx, ny))

    return []


def theta_star_with_fallback(
    start_world: Tuple[float, float],
    goal_world: Tuple[float, float],
    grid_data: List[int],
    info: MapInfo,
    robot_radius_m: float = 0.20,
    min_radius_m: float = 0.08,
) -> List[WayPoint]:
    
    path = theta_star(start_world, goal_world, grid_data, info, robot_radius_m)
    if path:
        return path

    radius = robot_radius_m - 0.04
    while radius >= min_radius_m - 1e-9:
        path = theta_star(start_world, goal_world, grid_data, info, radius)
        if path:
            return path
        radius -= 0.04

    return []


# ---------------------------------------------------------------------------
# 4. Follow-goal & replan helpers
# ---------------------------------------------------------------------------

def compute_follow_goal(actor_x: float, actor_y: float, actor_yaw: float,
                        follow_distance: float = 0.5) -> Tuple[float, float]:
    
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
   
    new_goal = compute_follow_goal(actor_x, actor_y, actor_yaw, follow_distance)
    dist = compute_distance(current_goal[0], current_goal[1],
                            new_goal[0], new_goal[1])
    return dist > move_threshold
