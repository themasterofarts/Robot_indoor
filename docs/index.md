---
layout: default
title: Robot_indoor
---

# Robot_indoor

Robot_indoor est une plateforme de développement ROS 2 pour la simulation de robots mobiles en environnement intérieur. Le projet est porté dans un esprit communautaire par MA64 Robotics et vise à offrir un support académique, simple à prendre en main et évolutif.

![Simulation de navigation indoor](assets/indoor_nav.png)

## Objectif

Le projet fournit une base de travail pour apprendre et expérimenter la robotique mobile indoor :

- simuler un robot mobile dans Gazebo ;
- exploiter des capteurs simulés comme le LiDAR, la caméra de profondeur, l'IMU et l'odométrie ;
- créer une carte 2D avec SLAM Toolbox ;
- lancer la navigation autonome avec Nav2 ;
- visualiser et piloter les scénarios avec RViz ;
- étendre progressivement la plateforme avec de nouveaux comportements.

## Contexte

Robot_indoor s'adresse aux étudiants, enseignants, développeurs et passionnés de robotique qui souhaitent travailler sur une plateforme de simulation reproductible. Le projet privilégie la compréhension des composants ROS 2 et la possibilité de faire évoluer l'architecture sans dépendre immédiatement d'un robot physique.

## Fonctionnalités principales

- Simulation indoor avec Gazebo Sim Harmonic.
- Description du robot avec URDF et Xacro.
- Pont ROS 2 et Gazebo via `ros_gz_bridge`.
- Cartographie avec SLAM Toolbox.
- Navigation autonome avec Nav2.
- Visualisation avec RViz.
- Exploration automatique de frontières.
- Environnement Docker et Devcontainer.

## Architecture générale

```text
robot_indoor/
├── robot_indoor/                 # Robot, mondes Gazebo, fichiers launch, RViz
├── indoor_navigation/            # SLAM, Nav2 et exploration
├── gazebo-ros-actor-plugin/      # Plugin Gazebo pour acteurs simulés
├── docker/                       # Image Docker et script de développement
├── .devcontainer/                # Configuration VS Code Devcontainer
├── doc/                          # Images et démonstrations
└── docs/                         # Documentation GitHub Pages
```

## Technologies

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Sim Harmonic
- Nav2
- SLAM Toolbox
- RViz2
- `ros_gz`, `ros_gz_bridge`, `ros_gz_image`
- URDF et Xacro
- Python et CMake
- Docker et Devcontainer

## Installation rapide avec Docker

```bash
git clone git@github.com:themasterofarts/Robot_indoor.git
cd Robot_indoor
git submodule update --init --recursive
./docker/run-dev.sh
```

Dans le conteneur :

```bash
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths . --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install
source install/setup.bash
```

## Installation native

Sur Ubuntu 24.04 avec ROS 2 Jazzy :

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-dev-tools \
  ros-jazzy-nav2-bringup \
  ros-jazzy-navigation2 \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-image \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-rviz2 \
  ros-jazzy-slam-toolbox \
  ros-jazzy-xacro
```

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone git@github.com:themasterofarts/Robot_indoor.git
cd Robot_indoor
git submodule update --init --recursive

cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install
source install/setup.bash
```

## Utilisation

Lancer la simulation :

```bash
ros2 launch robot_indoor view.launch.py
```

Créer une carte :

```bash
ros2 launch indoor_navigation mapping.launch.py
```

Lancer la navigation autonome :

```bash
ros2 launch indoor_navigation indoor_nav.launch.py
```

Lancer l'exploration de frontières :

```bash
ros2 launch indoor_navigation frontier_exploration.launch.py
```

## Exemples

Afficher les topics ROS 2 :

```bash
ros2 topic list
```

Envoyer une commande de vitesse simple :

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}" --once
```

Vérifier les packages du projet :

```bash
ros2 pkg list | grep -E "robot_indoor|indoor_navigation"
```

## Captures et schémas prévus

La documentation pourra être enrichie avec :

- un schéma global ROS 2, Gazebo, Nav2 et RViz ;
- une capture du robot dans Gazebo ;
- une capture de carte générée avec SLAM Toolbox ;
- une vidéo de navigation autonome ;
- un schéma des topics principaux.

## Publication GitHub Pages

Cette page est conçue pour être publiée depuis le dossier `docs/`.

```bash
git add README.md docs/
git commit -m "docs: add project documentation page"
git push origin main
```

Dans GitHub, ouvrir `Settings`, puis `Pages`, choisir `Deploy from a branch`, sélectionner `main` et le dossier `/docs`.

URL attendue :

```text
https://themasterofarts.github.io/Robot_indoor/
```

## Contribution et contact

Les contributions sont bienvenues : documentation, scénarios Gazebo, amélioration de la navigation, exploration, tests ou corrections de bugs.

- Dépôt GitHub : [themasterofarts/Robot_indoor](https://github.com/themasterofarts/Robot_indoor)
- Projet : MA64 Robotics
- Contact mainteneur : `klein <kleinfy51@gmail.com>`
