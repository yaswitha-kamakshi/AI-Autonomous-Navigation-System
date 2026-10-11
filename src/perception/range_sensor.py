"""Simulated range sensor (like ultrasonic/LiDAR): ray casting from the car."""
import math
from src.utils import config as cfg


def scan(env):
    """Return a list of (angle, distance) for each beam, fanned across the car's front."""
    car = env.car
    rects = env.all_obstacle_rects()
    n = cfg.RAY_COUNT
    readings = []
    for i in range(n):
        offset = -cfg.RAY_FOV / 2 + i * cfg.RAY_FOV / (n - 1)
        ang = car.heading + offset
        dx, dy = math.cos(ang), math.sin(ang)
        dist = cfg.RAY_MAX
        d = cfg.CAR_RADIUS                 # start just outside the car body
        while d <= cfg.RAY_MAX:
            px, py = car.x + dx * d, car.y + dy * d
            if any(r.collidepoint(px, py) for r in rects):
                dist = d
                break
            d += cfg.RAY_STEP
        readings.append((ang, dist))
    return readings