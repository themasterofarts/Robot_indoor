import math
import random


def straight_line(rng: random.Random, length: float = None, num_points: int = 50) -> list:
    """
    Génère un segment rectiligne le long de l'axe X.

    Args:
        rng        : instance Random isolée (n'affecte pas le RNG global)
        length     : longueur du segment en mètres (aléatoire si None)
        num_points : nombre de waypoints

    Returns:
        liste de tuples (x, y)
    """
    if length is None:
        length = rng.uniform(3.0, 6.0)

    step = length / num_points
    return [(i * step, 0.0) for i in range(num_points)]


def random_arc(
    rng: random.Random,
    radius: float = None,
    angle: float = None,
    direction: int = None,
    num_points: int = 80
) -> list:
    """
    Génère un arc de cercle.

    Args:
        rng        : instance Random isolée
        radius     : rayon de l'arc en mètres (aléatoire si None)
        angle      : angle total de l'arc en radians (aléatoire si None)
        direction  : 1 = gauche, -1 = droite (aléatoire si None)
        num_points : nombre de waypoints

    Returns:
        liste de tuples (x, y)
    """
    if radius is None:
        radius = rng.uniform(1.5, 3.5)
    if angle is None:
        angle = rng.uniform(math.pi / 3, math.pi)
    if direction is None:
        direction = rng.choice([-1, 1])

    points = []
    for i in range(num_points):
        t = angle * i / num_points
        x = radius * math.sin(t)
        y = direction * radius * (1 - math.cos(t))
        points.append((x, y))

    return points


def zig_zag(
    rng: random.Random,
    length: float = None,
    amplitude: float = 0.6,
    num_zigs: int = 5,
    num_points: int = 100
) -> list:
    """
    Génère un segment en zigzag avec transitions lissées (sinusoïde).

    CORRECTION : l'ancienne version utilisait abs(2t - 1) ce qui créait
    des pics anguleux (discontinuités de dérivée). Un robot différentiel
    à vitesse limitée ne peut pas suivre ces angles brusques.
    On utilise maintenant une sinusoïde qui est infiniment dérivable.

    L'amplitude est aussi réduite (1.0 → 0.6) pour rester réaliste.

    Args:
        rng        : instance Random isolée
        length     : longueur totale en mètres (aléatoire si None)
        amplitude  : hauteur des oscillations en mètres
        num_zigs   : nombre de périodes
        num_points : nombre de waypoints

    Returns:
        liste de tuples (x, y)
    """
    if length is None:
        length = rng.uniform(4.0, 7.0)

    points = []
    step = length / num_points

    for i in range(num_points):
        x = i * step
        # Sinusoïde au lieu de zigzag triangulaire → courbe lisse
        t = (x / length) * num_zigs * 2 * math.pi
        y = amplitude * math.sin(t)
        points.append((x, y))

    return points


def combined_path(seed: int = None) -> list:
    """
    Génère une trajectoire combinée en enchaînant plusieurs segments.

    Chaque segment est aligné et translaté pour former un chemin continu.
    Les segments sont choisis aléatoirement parmi : ligne droite, arc, zigzag.

    CORRECTION : on utilise random.Random(seed) au lieu de random.seed(seed).
    L'ancienne version modifiait le RNG global de Python, ce qui pouvait
    affecter d'autres parties du programme utilisant random. Maintenant
    le générateur est isolé dans cette fonction.

    Args:
        seed : graine pour la reproductibilité (optionnel)

    Returns:
        liste de tuples (x, y) représentant la trajectoire complète
    """
    # RNG isolé : n'affecte pas random.random() ailleurs dans le programme
    rng = random.Random(seed)

    # Pool de segments disponibles
    segment_pool = (
        [straight_line] * 2 +
        [random_arc]    * 3 +
        [zig_zag]       * 1
    )

    chosen = rng.sample(segment_pool, k=rng.randint(5, 6))

    full_path = []

    for seg_func in chosen:
        seg = seg_func(rng)

        if full_path:
            last_x    = full_path[-1][0]
            last_y    = full_path[-1][1]
            prev_x    = full_path[-2][0]
            prev_y    = full_path[-2][1]
            angle_out = math.atan2(last_y - prev_y, last_x - prev_x)

            angle_in = math.atan2(seg[1][1] - seg[0][1], seg[1][0] - seg[0][0])

            rotation = angle_out - angle_in
            cos_r    = math.cos(rotation)
            sin_r    = math.sin(rotation)

            rotated = [(cos_r * x - sin_r * y, sin_r * x + cos_r * y) for x, y in seg]

            ox  = last_x - rotated[0][0]
            oy  = last_y - rotated[0][1]
            seg = [(x + ox, y + oy) for x, y in rotated]

            seg = seg[1:]

        full_path.extend(seg)

    return full_path
