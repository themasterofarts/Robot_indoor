---
layout: default
title: Robot_indoor
---

# Robot_indoor

Robot_indoor est une plateforme académique et communautaire de simulation robotique portée par MA64 Robotics. Son ambition est de fournir un environnement clair, évolutif et accessible pour comprendre, tester et développer des comportements de robots mobiles en intérieur.

![Simulation de navigation indoor](assets/indoor_nav.png)

## Vision du projet

La robotique mobile demande de relier plusieurs domaines qui sont souvent étudiés séparément : modélisation, simulation, perception, cartographie, localisation, navigation, visualisation et interaction avec l'environnement.

Robot_indoor cherche à réunir ces briques dans une plateforme unique, suffisamment simple pour apprendre, mais suffisamment ouverte pour évoluer vers des scénarios plus avancés.

Le projet vise trois objectifs principaux :

- offrir un support pédagogique pour découvrir la robotique mobile avec ROS 2 ;
- permettre l'expérimentation sans dépendre immédiatement d'un robot physique ;
- construire progressivement une base commune réutilisable par la communauté MA64 Robotics.

## Pourquoi la simulation

La simulation permet de tester rapidement des idées, de reproduire des scénarios et d'observer le comportement du robot dans un environnement contrôlé. Elle est particulièrement utile pour un projet académique, car elle réduit les contraintes matérielles tout en conservant une logique proche d'un système robotique réel.

Avec Gazebo, ROS 2, Nav2 et SLAM Toolbox, Robot_indoor permet d'aborder les étapes essentielles d'un robot mobile autonome :

- percevoir l'environnement avec des capteurs simulés ;
- construire ou utiliser une carte ;
- estimer la position du robot ;
- planifier une trajectoire ;
- exécuter un déplacement ;
- observer et analyser le résultat dans RViz.

## Public visé

Le projet s'adresse principalement à :

- des étudiants souhaitant comprendre ROS 2 et la navigation mobile ;
- des enseignants cherchant un support de travaux pratiques ;
- des développeurs voulant tester des algorithmes dans un cadre reproductible ;
- des passionnés de robotique qui souhaitent s'exercer sans robot physique ;
- des contributeurs MA64 Robotics voulant enrichir une base commune.

## Principe général

Robot_indoor s'appuie sur une architecture modulaire. Chaque composant joue un rôle précis dans la chaîne robotique.

```text
Simulation Gazebo
        |
        v
Robot simulé + capteurs
        |
        v
Pont ROS 2 / Gazebo
        |
        v
SLAM, localisation, navigation
        |
        v
Visualisation et commandes dans RViz
```

Cette organisation aide à comprendre comment les informations circulent entre le simulateur, les capteurs, les topics ROS 2 et les noeuds de navigation.

## Composants du projet

Le dépôt est construit autour de plusieurs blocs :

- `robot_indoor` : description du robot, mondes Gazebo, fichiers de lancement, configuration RViz et ponts ROS/Gazebo ;
- `indoor_navigation` : configuration de cartographie, navigation autonome et exploration ;
- `gazebo-ros-actor-plugin` : extension Gazebo pour intégrer des acteurs simulés ;
- `docker` et `.devcontainer` : environnement reproductible pour développer plus facilement ;
- `docs` : documentation web et ressources de présentation.

Le README du dépôt reste le document de référence pour l'installation, les commandes de lancement et les exemples techniques.

## Ce que le projet permet aujourd'hui

Robot_indoor permet actuellement de :

- lancer une simulation indoor avec un robot mobile ;
- afficher le robot et son environnement dans Gazebo ;
- publier les données de capteurs vers ROS 2 ;
- créer une carte avec SLAM Toolbox ;
- utiliser Nav2 pour la navigation autonome ;
- visualiser les informations principales dans RViz ;
- lancer une exploration basée sur les frontières ;
- travailler dans un environnement Docker ou Devcontainer.

## Plan d'évolution

Robot_indoor est pensé comme un projet progressif. Les étapes suivantes peuvent guider son développement.

### Étape 1 : stabiliser la base

- consolider les fichiers de lancement ;
- clarifier les dépendances ROS 2 ;
- documenter les scénarios de base ;
- vérifier le fonctionnement Docker et Devcontainer ;
- maintenir une intégration continue fiable.

### Étape 2 : améliorer la compréhension pédagogique

- ajouter des schémas de flux ROS 2 ;
- expliquer les topics et frames principaux ;
- documenter le rôle des capteurs simulés ;
- proposer des exercices simples ;
- ajouter des captures commentées de Gazebo, RViz et Nav2.

### Étape 3 : enrichir les scénarios de simulation

- créer plusieurs mondes indoor ;
- ajouter des obstacles dynamiques ;
- intégrer des acteurs simulés plus réalistes ;
- varier les conditions de navigation ;
- proposer des scénarios de test reproductibles.

### Étape 4 : développer l'autonomie

- améliorer l'exploration automatique ;
- tester plusieurs stratégies de navigation ;
- intégrer des comportements de suivi ou d'évitement ;
- comparer les performances selon les cartes et paramètres ;
- préparer des démonstrations complètes.

### Étape 5 : préparer l'ouverture vers le réel

- identifier les écarts entre simulation et robot physique ;
- isoler les paramètres dépendants du matériel ;
- préparer une architecture transférable ;
- documenter les contraintes de capteurs, odométrie et calibration.

## Axes de contribution

Les contributions peuvent porter sur plusieurs aspects :

- documentation et supports pédagogiques ;
- amélioration des mondes Gazebo ;
- ajout ou configuration de capteurs ;
- navigation et exploration ;
- visualisation RViz ;
- tests, validation et intégration continue ;
- scénarios de démonstration ;
- préparation d'une future transition vers un robot réel.

L'objectif n'est pas seulement d'ajouter du code, mais aussi de rendre le projet plus compréhensible, plus fiable et plus utile pour apprendre.

## Ressources

- Dépôt GitHub : [themasterofarts/Robot_indoor](https://github.com/themasterofarts/Robot_indoor)
- Documentation technique : [README du projet](https://github.com/themasterofarts/Robot_indoor#readme)
- Démonstration vidéo : [navigation indoor](assets/indoor_nav_v.mp4)
- Projet : MA64 Robotics

## Esprit du projet

Robot_indoor est un point de départ. Il doit rester lisible, modulaire et accueillant pour les nouveaux contributeurs. Chaque amélioration doit aider le lecteur ou le développeur suivant à mieux comprendre la robotique mobile, à expérimenter plus facilement et à construire sur une base commune.
