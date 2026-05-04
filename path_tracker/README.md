# Path Tracker — Stanley Controller

Algorithme : Stanley Controller
Robot cible : Differentiel (deux roues motrices independantes)


## Objectif

Module Python de suivi de trajectoire pour robot differentiel.
Prend en entree une trajectoire (liste de points 2D) et retourne
les vitesses des roues gauche et droite a chaque instant.


## Pourquoi Stanley 

Stanley Controller corrige simultanement deux erreurs :

- Erreur de cap : difference d'angle entre le robot et la trajectoire
- Erreur laterale (CTE) : distance perpendiculaire entre le robot et la trajectoire


Stanley est plus precis sur les trajectoires complexes comme les virages serres et le zig zag.

Formule principale :

    delta = heading_error + arctan(k * CTE / v)


## Structure du projet

    path_tracker/
    base_tracker.py   : Interface commune a tous les algorithmes
    robot_model.py    : Modele cinematique differentiel
    trajectory.py     : Generateur de trajectoires (arc, ligne droite, zig zag)
    stanley.py        : Implementation Stanley Controller
    simulator.py      : Simulation et visualisation matplotlib
    result.png          : Resultat de simulation
    README.md         : Ce fichier


## Utilisation rapide

    from stanley import StanleyTracker

    tracker = StanleyTracker(k=0.75, v_base=0.15, v_max=0.5, wheel_base=0.3)

    robot_state = {'x': 0.0, 'y': 0.0, 'theta': 0.0, 'v': 0.0}
    trajectory  = [(0.0, 0.0), (1.0, 0.5), (2.0, 1.0)]

    v_gauche, v_droite = tracker.compute_velocities(robot_state, trajectory, 0)


## Lancer la simulation

    pip install matplotlib
    python3 simulator.py

Le resultat est sauvegarde dans result.png.


## Parametres Stanley

    k          = 0.75       Gain correction laterale
    v_base     = 0.15 m/s  Vitesse nominale
    v_max      = 0.50 m/s  Vitesse maximale
    wheel_base = 0.30 m    Distance entre les deux roues


## Modele du robot

Le robot est represente a chaque instant par son etat :

    robot_state = {
        'x'     : position X en metres
        'y'     : position Y en metres
        'theta' : orientation en radians
        'v'     : vitesse lineaire courante
    }

Les vitesses de roues sont calculees ainsi :

    v_lineaire  = (v_droite + v_gauche) / 2
    v_angulaire = (v_droite - v_gauche) / wheel_base


## Trajectoires supportees

    Ligne droite   : segment rectiligne de longueur aleatoire
    Arc de cercle  : rayon et angle balaye choisis aleatoirement
    Zig zag        : amplitude et nombre de dents parametrables
    Chemin combine : enchainent plusieurs segments avec jonction lisse


## Resultat

Le robot (rouge) suit fidelement la trajectoire cible (bleu)
sur des chemins combinant arcs de cercle, lignes droites et zig zag.

Voir result.png.
