# Docker et Devcontainer

Le `Dockerfile` de ce dossier sert de base unique pour :

- construire l'image locale du projet ;
- ouvrir le depot dans un devcontainer ;
- lancer un conteneur interactif avec Gazebo / RViz via X11.

## Construire l'image

```bash
docker build -f docker/Dockerfile -t robot-indoor:jazzy .
```

## Lancer le conteneur avec interface graphique

```bash
./docker/run-dev.sh
```

Exemple pour lancer directement un build ROS 2 dans le conteneur :

```bash
./docker/run-dev.sh bash -lc "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"
```

## Devcontainer

Le fichier `.devcontainer/devcontainer.json` reutilise ce meme `Dockerfile`.

A l'ouverture du conteneur, la commande `postCreateCommand` :

- met a jour `rosdep` ;
- installe les dependances ROS du depot ;
- lance `colcon build --symlink-install`.
