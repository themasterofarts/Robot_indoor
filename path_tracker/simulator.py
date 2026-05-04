import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from robot_model import DifferentialRobot
from stanley import StanleyTracker
from trajectory import combined_path

DT          = 0.05
MAX_STEPS   = 6000
GOAL_RADIUS = 0.1
SEED        = 45

trajectory = combined_path(seed=SEED)

robot   = DifferentialRobot(wheel_base=0.3)
tracker = StanleyTracker(k=0.75, v_base=0.15, v_max=0.5, wheel_base=0.3)

dx0 = trajectory[1][0] - trajectory[0][0]
dy0 = trajectory[1][1] - trajectory[0][1]

robot_state = {
    'x'    : trajectory[0][0],
    'y'    : trajectory[0][1],
    'theta': math.atan2(dy0, dx0),
    'v'    : 0.0
}

current_index = 0
history_x     = [robot_state['x']]
history_y     = [robot_state['y']]

for step in range(MAX_STEPS):
    if current_index >= len(trajectory) - 1:
        print("Arrivée !")
        break

    v_left, v_right = tracker.compute_velocities(
        robot_state, trajectory, current_index
    )

    robot_state = robot.update(robot_state, v_left, v_right, DT)

    dx   = trajectory[current_index][0] - robot_state['x']
    dy   = trajectory[current_index][1] - robot_state['y']
    dist = math.sqrt(dx**2 + dy**2)

    if dist < GOAL_RADIUS:
        current_index = min(current_index + 1, len(trajectory) - 1)

    history_x.append(robot_state['x'])
    history_y.append(robot_state['y'])

traj_x = [p[0] for p in trajectory]
traj_y = [p[1] for p in trajectory]

fig, ax = plt.subplots(figsize=(10, 6))
ax.plot(traj_x, traj_y, 'b-', linewidth=2, label='Trajectoire cible')
ax.plot(history_x, history_y, 'r-', linewidth=1.5, label='Chemin robot')
ax.plot(history_x[0], history_y[0], 'go', markersize=10, label='Départ')
ax.plot(history_x[-1], history_y[-1], 'rs', markersize=10, label='Arrivée')
ax.set_aspect('equal')
ax.legend()
ax.set_title('Stanley Controller — Path Tracking')
ax.grid(True)
plt.tight_layout()
plt.savefig('result.png', dpi=150)
print("Image sauvegardée : result.png")
