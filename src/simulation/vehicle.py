"""Virtual car using the kinematic bicycle model."""
import math
from src.utils import config as cfg


class Vehicle:
    def __init__(self, x, y, heading):
        self.x, self.y, self.heading = x, y, heading
        self.speed = 0.0
        self.steer = 0.0
        self._prev = (x, y, heading)

    def update(self, throttle, steer_cmd, dt):
        """throttle in [-1, 1]; steer_cmd in radians. Screen y points DOWN,
        so a positive heading change turns the car clockwise (to the right)."""
        self._prev = (self.x, self.y, self.heading)
        self.steer = max(-cfg.MAX_STEER, min(cfg.MAX_STEER, steer_cmd))

        # Speed update
        if throttle > 0:
            self.speed += cfg.ACCEL * throttle * dt
        elif throttle < 0:
            self.speed += cfg.BRAKE * throttle * dt
        else:  # coast: friction pulls speed toward zero
            drag = cfg.FRICTION * dt
            if abs(self.speed) <= drag:
                self.speed = 0.0
            else:
                self.speed -= math.copysign(drag, self.speed)
        self.speed = max(-cfg.MAX_REVERSE, min(cfg.MAX_SPEED, self.speed))

        # Bicycle model motion update
        self.x += self.speed * math.cos(self.heading) * dt
        self.y += self.speed * math.sin(self.heading) * dt
        self.heading += (self.speed / cfg.WHEELBASE) * math.tan(self.steer) * dt

    def revert(self):
        """Undo the last move (used after a collision)."""
        self.x, self.y, self.heading = self._prev
        self.speed = 0.0

    def corners(self):
        """Four corner points of the car body, for drawing."""
        c, s = math.cos(self.heading), math.sin(self.heading)
        half_l, half_w = cfg.CAR_LENGTH / 2, cfg.CAR_WIDTH / 2
        pts = []
        for lx, ly in [(half_l, half_w), (half_l, -half_w), (-half_l, -half_w), (-half_l, half_w)]:
            pts.append((self.x + lx * c - ly * s, self.y + lx * s + ly * c))
        return pts