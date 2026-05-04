import math
import random


def straight_line(length=None, num_points=50):
    if length is None:
        length = random.uniform(3.0, 6.0)
    step = length / num_points
    return [(i * step, 0.0) for i in range(num_points)]


def random_arc(radius=None, angle=None, direction=None, num_points=80):
    if radius is None:
        radius = random.uniform(1.5, 3.5)
    if angle is None:
        angle = random.uniform(math.pi / 3, math.pi)
    if direction is None:
        direction = random.choice([-1, 1])

    points = []
    for i in range(num_points):
        t = angle * i / num_points
        x = radius * math.sin(t)
        y = direction * radius * (1 - math.cos(t))
        points.append((x, y))

    return points


def zig_zag(length=None, amplitude=1.0, num_zigs=5, num_points=100):
    if length is None:
        length = random.uniform(4.0, 7.0)
    points = []
    step = length / num_points
    zig_width = length / num_zigs

    for i in range(num_points):
        x = i * step
        t = (x % zig_width) / zig_width
        y = amplitude * (1 - abs(2 * t - 1))
        points.append((x, y))

    return points


def combined_path(seed=None):
    if seed is not None:
        random.seed(seed)

    # 2 à 3 segments de chaque type, plus courts
    segment_pool = (
        [straight_line] * 2 +
        [random_arc] * 3 +
        [zig_zag] * 1
    )

    chosen = random.sample(segment_pool, k=random.randint(5, 6))

    full_path = []

    for seg_func in chosen:
        seg = seg_func()

        if full_path:
            last_x = full_path[-1][0]
            last_y = full_path[-1][1]

            prev_x = full_path[-2][0]
            prev_y = full_path[-2][1]
            angle_out = math.atan2(last_y - prev_y, last_x - prev_x)

            first_x = seg[0][0]
            first_y = seg[0][1]
            second_x = seg[1][0]
            second_y = seg[1][1]
            angle_in = math.atan2(second_y - first_y, second_x - first_x)

            rotation = angle_out - angle_in
            cos_r = math.cos(rotation)
            sin_r = math.sin(rotation)

            rotated = []
            for x, y in seg:
                xr = cos_r * x - sin_r * y
                yr = sin_r * x + cos_r * y
                rotated.append((xr, yr))

            ox = last_x - rotated[0][0]
            oy = last_y - rotated[0][1]
            seg = [(x + ox, y + oy) for x, y in rotated]
            seg = seg[1:]

        full_path.extend(seg)

    return full_path
