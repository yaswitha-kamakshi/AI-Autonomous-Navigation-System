"""The autonomy loop: perceive -> plan -> control, with a simple state machine."""
import math
from enum import Enum
from src.utils import config as cfg
from src.perception.camera import capture
from src.perception.detector import detect
from src.perception.occupancy_grid import OccupancyGrid
from src.perception.range_sensor import scan
from src.planning.replanner import plan_route, path_blocked
from src.control.controller import PurePursuitController
from src.control.safety import SafetyMonitor
from src.simulation.obstacles import StaticObstacle


class State(Enum):
    IDLE = "IDLE"
    NAVIGATING = "NAVIGATING"
    AVOIDING = "AVOIDING"
    RECOVERING = "RECOVERING"
    REPLANNING = "REPLANNING"
    GOAL_REACHED = "GOAL_REACHED"
    FAILED = "FAILED"


class AutonomousAgent:
    def __init__(self, env):
        self.env = env
        self.controller = PurePursuitController()
        self.safety = SafetyMonitor()
        self.state = State.IDLE
        self.grid = None
        self.detections = []
        self.readings = []
        self.status = None
        self.path = []
        self.replans = 0
        self.no_route = False
        self.last_plan_time = -99.0
        self.frame = 0
        self.target_speed = 0.0
        self.stuck_for = 0.0
        self.recover_until = -1.0
        self.recoveries = 0

    def _perceive(self):
        self.detections, mask = detect(capture(self.env))
        self.grid = OccupancyGrid(mask)

    def _plan(self):
        car = self.env.car
        route = plan_route(self.grid, (car.x, car.y), cfg.GOAL_POS)
        self.last_plan_time = self.env.time
        if route is None:
            return False
        self.path = route
        self.controller.set_path(route)
        return True

    def step(self):
        """Decide the next action. Returns (throttle, steer_angle)."""
        env, car = self.env, self.env.car
        if env.goal_reached:
            self.state = State.GOAL_REACHED
            return 0.0, 0.0
        if env.time > cfg.MAX_RUN_TIME:
            self.state = State.FAILED
            return 0.0, 0.0

        if self.grid is None or self.frame % cfg.PERCEPTION_EVERY == 0:
            self._perceive()
        self.frame += 1
        self.readings = scan(env)
        self.status = self.safety.evaluate(self.readings, car.heading, car.speed)

        # --- Recovery: back up when wedged against something, then replan ---
        if self.state == State.RECOVERING:
            if env.time < self.recover_until:
                return -0.7, 0.0
            self.last_plan_time = -99.0          # allow an immediate replan
            self.state = State.NAVIGATING
            self.stuck_for = 0.0

        if self.state == State.IDLE:
            self.state = State.NAVIGATING if self._plan() else State.FAILED
            if self.state == State.FAILED:
                return 0.0, 0.0
        else:
            blocked = path_blocked(self.grid, self.controller.path, self.controller.idx,
                                   (car.x, car.y), cfg.PATH_CHECK_PX)
            if self.no_route and not blocked:
                self.no_route = False
            if blocked and env.time - self.last_plan_time > cfg.REPLAN_COOLDOWN:
                self.state = State.REPLANNING
                if self._plan():
                    self.replans += 1
                    self.no_route = False
                else:
                    self.no_route = True    # stop and retry after the cooldown
                self.state = State.NAVIGATING

        if self.state == State.NAVIGATING and (self.status.emergency or self.no_route):
            self.state = State.AVOIDING
        elif self.state == State.AVOIDING and not self.status.emergency and not self.no_route:
            self.state = State.NAVIGATING

        cap = 0.0 if self.no_route else self.status.speed_cap
        throttle, steer, self.target_speed = self.controller.compute(
            car, env.distance_to_goal(), cap, cfg.DT)

        # Stuck detection: we want to move but the car is not moving.
        if self.target_speed > 15 and abs(car.speed) < 3:
            self.stuck_for += cfg.DT
        else:
            self.stuck_for = 0.0
        if self.stuck_for > cfg.STUCK_TIME:
            self.state = State.RECOVERING
            self.recover_until = env.time + cfg.RECOVER_TIME
            self.recoveries += 1
            self.controller.speed_pid.reset()
            return -0.7, 0.0
        return throttle, steer


def drop_block_on_path(env, agent, ahead_px=220, size=60):
    """Demo helper: place a surprise obstacle on the route ahead of the car."""
    path = agent.controller.path
    if not path:
        return False
    i = min(len(path) - 1, agent.controller.idx + int(ahead_px / cfg.PATH_SPACING))
    x, y = path[i]
    if math.hypot(x - cfg.GOAL_POS[0], y - cfg.GOAL_POS[1]) < 120:
        return False                       # never block the goal itself
    env.static_obstacles.append(StaticObstacle(x - size / 2, y - size / 2, size, size))
    return True