"""Pure-pursuit steering and PID speed control."""
import math
from src.utils import config as cfg


def wrap(angle):
    """Wrap an angle to [-pi, pi]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


class PID:
    def __init__(self, kp, ki, kd, i_limit=2.0):
        self.kp, self.ki, self.kd, self.i_limit = kp, ki, kd, i_limit
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev = None

    def update(self, error, dt):
        self.integral = max(-self.i_limit, min(self.i_limit, self.integral + error * dt))
        deriv = 0.0 if self.prev is None else (error - self.prev) / dt
        self.prev = error
        return self.kp * error + self.ki * self.integral + self.kd * deriv


def densify(points, spacing):
    """Insert points so the path has a waypoint every `spacing` pixels."""
    if not points:
        return []
    out = [points[0]]
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        seg = math.hypot(x1 - x0, y1 - y0)
        n = max(1, int(seg // spacing))
        for i in range(1, n + 1):
            t = i / n
            out.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))
    return out


class PurePursuitController:
    def __init__(self):
        self.path = []
        self.idx = 0
        self.speed_pid = PID(cfg.PID_KP, cfg.PID_KI, cfg.PID_KD)

    def set_path(self, waypoints):
        self.path = densify(waypoints, cfg.PATH_SPACING)
        self.idx = 0
        self.speed_pid.reset()

    def _lookahead_point(self, x, y, ld):
        # Find the closest path point (searching only forward, so we never go back).
        best, best_d = self.idx, float("inf")
        for i in range(self.idx, min(self.idx + 40, len(self.path))):
            d = math.hypot(self.path[i][0] - x, self.path[i][1] - y)
            if d < best_d:
                best, best_d = i, d
        self.idx = best
        j = best
        while j < len(self.path) - 1 and math.hypot(self.path[j][0] - x, self.path[j][1] - y) < ld:
            j += 1
        return self.path[j]

    def compute(self, car, dist_to_goal, speed_cap, dt):
        """Return (throttle, steer_angle, target_speed)."""
        if not self.path:
            return 0.0, 0.0, 0.0
        ld = max(cfg.LOOKAHEAD_MIN,
                 min(cfg.LOOKAHEAD_MAX, cfg.LOOKAHEAD_MIN + cfg.LOOKAHEAD_GAIN * abs(car.speed)))
        px, py = self._lookahead_point(car.x, car.y, ld)
        alpha = wrap(math.atan2(py - car.y, px - car.x) - car.heading)
        steer = math.atan2(2.0 * cfg.WHEELBASE * math.sin(alpha), ld)

        # Target speed: slow in sharp turns, slow near the goal, obey the safety cap.
        target = cfg.CRUISE_SPEED * (1.0 - 0.6 * min(1.0, abs(alpha) / 0.8))
        target = min(target, max(cfg.MIN_SPEED, cfg.GOAL_SLOW_GAIN * dist_to_goal))
        target = min(target, speed_cap)

        throttle = max(-1.0, min(1.0, self.speed_pid.update(target - car.speed, dt)))
        if throttle < 0:   # never brake so hard that the car reverses
            limit = max(0.0, car.speed) / (cfg.BRAKE * dt)
            throttle = -min(-throttle, limit, 1.0)
        return throttle, steer, target