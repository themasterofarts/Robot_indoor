"""
Simulateur interne pour valider StanleyController avec la même interface
que PathController. Ce fichier est uniquement pour les tests.
"""
import math
import datetime
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from .stanley_controller import (
    StanleyController, StanleyConfig,
    Pose2D, Twist2D, TrajectoryPoint, ControlStatus
)
from .trajectory import combined_path

MAX_OMEGA  = 2.0
WHEEL_BASE = 0.297
DT         = 0.05
MAX_STEPS  = 8000
SEED       = 45
LOG_EVERY  = 100

def robot_update(pose: Pose2D, twist: Twist2D, dt: float) -> Pose2D:
    v     = twist.linear
    omega = max(-MAX_OMEGA, min(MAX_OMEGA, twist.angular))
    new_x   = pose.x + v * math.cos(pose.yaw) * dt
    new_y   = pose.y + v * math.sin(pose.yaw) * dt
    new_yaw = math.atan2(math.sin(pose.yaw + omega * dt), math.cos(pose.yaw + omega * dt))
    return Pose2D(x=new_x, y=new_y, yaw=new_yaw)

raw_path   = combined_path(seed=SEED)
trajectory = [TrajectoryPoint(x=p[0], y=p[1]) for p in raw_path]
print(f"Trajectoire générée : {len(trajectory)} waypoints")

config     = StanleyConfig(k=0.75, wheel_base=WHEEL_BASE)
controller = StanleyController(config=config)
controller.set_plan(trajectory)

dx0  = trajectory[1].x - trajectory[0].x
dy0  = trajectory[1].y - trajectory[0].y
pose = Pose2D(x=trajectory[0].x, y=trajectory[0].y, yaw=math.atan2(dy0, dx0))

history_x    = [pose.x]
history_y    = [pose.y]
success      = False
final_status = ControlStatus.RUNNING
cmd          = None

for step in range(MAX_STEPS):
    cmd  = controller.compute_command(pose, DT)
    pose = robot_update(pose, cmd.twist, DT)
    history_x.append(pose.x)
    history_y.append(pose.y)

    if step % LOG_EVERY == 0:
        print(
            f"Step {step:5d} | "
            f"waypoint {cmd.target_index:4d}/{len(trajectory)-1} | "
            f"status : {cmd.status.value:12s} | "
            f"distance au but : {cmd.distance_to_goal:.2f} m"
        )

    if cmd.status == ControlStatus.GOAL_REACHED:
        success      = True
        final_status = cmd.status
        break
    if cmd.status in (ControlStatus.STUCK, ControlStatus.NO_PLAN):
        final_status = cmd.status
        break

if success:
    print(f"\n✅ Arrivée ! Trajectoire complète en {step} steps.")
else:
    print(f"\n❌ Échec : status={final_status.value} au step {step}, à {cmd.distance_to_goal:.2f} m du but.")

traj_x = [p.x for p in trajectory]
traj_y = [p.y for p in trajectory]

fig, ax = plt.subplots(figsize=(10, 6))
ax.plot(traj_x,    traj_y,    'b-', linewidth=2,   label='Trajectoire cible')
ax.plot(history_x, history_y, 'r-', linewidth=1.5, label='Chemin robot')
ax.plot(history_x[0],  history_y[0],  'go', markersize=10, label='Départ')
ax.plot(history_x[-1], history_y[-1], 'rs', markersize=10, label='Arrivée' if success else 'Arrêt')
ax.set_aspect('equal')
ax.legend()
ax.set_title(f'StanleyController (format Nav2) — {"Succès" if success else "Échec"} (seed={SEED})')
ax.grid(True)
plt.tight_layout()
timestamp   = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
output_file = f'result_stanley_nav2_seed{SEED}_{timestamp}.png'
plt.savefig(output_file, dpi=150)
print(f"Image sauvegardée : {output_file}")
