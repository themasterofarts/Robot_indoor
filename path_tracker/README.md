# Path Tracker — Stanley Controller

Algorithme : Stanley Controller
Robot cible : Differentiel (robot indoor MA64 Robotics)
Wheel base  : 0.297 m (extrait de robot_core_04.xacro)

## Objectif

Module de suivi de trajectoire base sur le controleur Stanley pour robot
differentiel. Fournit deux niveaux d utilisation :

1. Simulation standalone (matplotlib) pour valider l algorithme
2. Noeud ROS 2 pour suivi d acteur en temps reel dans Gazebo

## Pourquoi Stanley ?

Stanley Controller corrige simultanement deux erreurs :
- Erreur de cap : difference d angle entre le robot et la trajectoire
- Erreur laterale (CTE) : distance perpendiculaire entre le robot et la trajectoire

Formule principale :
    delta = heading_error + arctan(k * CTE / v)
    omega = v * tan(delta) / wheel_base

Stanley est adapte aux trajectoires complexes
car il corrige simultanement cap et position laterale.

## Structure du projet

    path_tracker/
    base_tracker.py          : Interface abstraite commune (Pose2D, Twist2D)
    robot_model.py           : Modele cinematique unicycle
    trajectory.py            : Generateur de trajectoires de test
    stanley_controller.py    : Controleur Stanley (interface Nav2-compatible)
    simulate_stanley.py      : Simulation et visualisation matplotlib
    README.md                : Ce fichier

    indoor_navigation/indoor_navigation/
    stanley_controller.py    : Copie pour import ROS 2
    stanley_node.py          : Noeud ROS 2 pour suivi d acteur

## Lancer la simulation standalone (matplotlib)

    cd Robot_indoor/Robot_indoor
    python3 -m path_tracker.simulate_stanley

Resultat attendu :
    Arrivee ! Trajectoire complete en 1213 steps.
    Image sauvegardee : result_stanley_nav2_seed45_*.png

## Lancer le suivi d acteur dans Gazebo

Terminal 1 - Gazebo :
    ros2 launch robot_indoor view.launch.py

Terminal 2 - Stanley node :
    ros2 run indoor_navigation stanley_node.py

Terminal 3 - Teleoperation de l acteur :
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/actor/cmd_vel

Le robot suit automatiquement l acteur avec :
- Distance de securite : 0.6 m (arret si trop proche)
- Fil d Ariane : waypoints enregistres tous les 0.25 m
- Frequence de controle : 20 Hz

## Interface StanleyController (compatible PathController)

    from path_tracker.stanley_controller import (
        StanleyController, StanleyConfig,
        Pose2D, TrajectoryPoint, ControlStatus
    )

    config     = StanleyConfig(k=0.75, wheel_base=0.297)
    controller = StanleyController(config=config)

    controller.set_plan([TrajectoryPoint(x=1.0, y=0.0)])

    cmd = controller.compute_command(pose, dt=0.05)
    # cmd.twist.linear  : vitesse lineaire (m/s)
    # cmd.twist.angular : vitesse angulaire (rad/s)
    # cmd.status        : RUNNING / GOAL_REACHED / STUCK / NO_PLAN

## Parametres Stanley

    k                  = 0.75      Gain correction laterale
    wheel_base         = 0.297 m   Distance entre les roues (URDF)
    max_linear_speed   = 0.8 m/s   Vitesse maximale
    max_angular_speed  = 1.5 rad/s Vitesse angulaire maximale
    goal_xy_tolerance  = 0.20 m    Tolerance d arrivee
    safety_distance    = 0.6 m     Distance de securite acteur

## Dimensions du robot (robot_core_04.xacro)

    wheel_radius   = 0.06 m
    wheel_offset_y = 0.1485 m
    wheel_base     = 2 * 0.1485 = 0.297 m
