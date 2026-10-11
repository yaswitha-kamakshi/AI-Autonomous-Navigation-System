"""The simulated world: map, obstacles, car, collisions, rendering."""
import math
import pygame
from src.utils import config as cfg
from src.simulation.vehicle import Vehicle
from src.simulation.obstacles import StaticObstacle, MovingObstacle


def circle_hits_rect(cx, cy, r, rect):
    """Circle-vs-rectangle collision test."""
    nx = max(rect.left, min(cx, rect.right))
    ny = max(rect.top, min(cy, rect.bottom))
    return (cx - nx) ** 2 + (cy - ny) ** 2 < r * r


class Environment:
    def __init__(self):
        self.reset()

    def reset(self):
        t = cfg.WALL_THICKNESS
        self.walls = [
            pygame.Rect(0, 0, cfg.WIDTH, t),
            pygame.Rect(0, cfg.HEIGHT - t, cfg.WIDTH, t),
            pygame.Rect(0, 0, t, cfg.HEIGHT),
            pygame.Rect(cfg.WIDTH - t, 0, t, cfg.HEIGHT),
        ]
        self.static_obstacles = [
            StaticObstacle(200, 400, 160, 40),
            StaticObstacle(420, 180, 40, 260),
            StaticObstacle(600, 480, 200, 50),
            StaticObstacle(700, 220, 50, 180),
            StaticObstacle(220, 150, 140, 40),
        ]
        self.moving_obstacles = [
            MovingObstacle(a=(250, 560), b=(550, 560), size=40, speed=70),
            MovingObstacle(a=(560, 120), b=(560, 380), size=36, speed=60),
        ]
        self.car = Vehicle(*cfg.START_POS, cfg.START_HEADING)
        self.collisions = 0
        self._in_collision = False
        self.goal_reached = False
        self.time = 0.0

    def all_obstacle_rects(self):
        rects = list(self.walls)
        rects += [o.rect for o in self.static_obstacles]
        rects += [o.rect for o in self.moving_obstacles]
        return rects

    def distance_to_goal(self):
        return math.hypot(self.car.x - cfg.GOAL_POS[0], self.car.y - cfg.GOAL_POS[1])

    def step(self, throttle, steer_cmd, dt=cfg.DT):
        if self.goal_reached:
            return
        self.time += dt
        for o in self.moving_obstacles:
            o.update(dt)
        self.car.update(throttle, steer_cmd, dt)

        hit = any(circle_hits_rect(self.car.x, self.car.y, cfg.CAR_RADIUS, r)
                  for r in self.all_obstacle_rects())
        if hit:
            if not self._in_collision:      # count each crash once
                self.collisions += 1
            self._in_collision = True
            self.car.revert()
        else:
            self._in_collision = False

        if self.distance_to_goal() < cfg.GOAL_RADIUS:
            self.goal_reached = True

    def render(self, screen, font):
        screen.fill(cfg.BG)
        for w in self.walls:
            pygame.draw.rect(screen, cfg.WALL, w)
        for o in self.static_obstacles + self.moving_obstacles:
            o.draw(screen)

        pygame.draw.circle(screen, cfg.START, (int(cfg.START_POS[0]), int(cfg.START_POS[1])), 16, 2)
        pygame.draw.circle(screen, cfg.GOAL, (int(cfg.GOAL_POS[0]), int(cfg.GOAL_POS[1])), cfg.GOAL_RADIUS, 3)

        pygame.draw.polygon(screen, cfg.CAR, self.car.corners())
        nose = (self.car.x + 24 * math.cos(self.car.heading),
                self.car.y + 24 * math.sin(self.car.heading))
        pygame.draw.line(screen, (255, 255, 255), (self.car.x, self.car.y), nose, 3)

        status = "GOAL REACHED!" if self.goal_reached else "DRIVING"
        lines = [f"Status    : {status}",
                 f"Speed     : {self.car.speed:6.1f}",
                 f"To goal   : {self.distance_to_goal():6.1f}",
                 f"Collisions: {self.collisions}",
                 f"Time      : {self.time:6.1f}s"]
        for i, text in enumerate(lines):
            screen.blit(font.render(text, True, cfg.TEXT), (24, 20 + i * 22))