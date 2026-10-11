"""Static and moving obstacles."""
import math
import pygame
from src.utils import config as cfg


class StaticObstacle:
    def __init__(self, x, y, w, h):
        self.rect = pygame.Rect(x, y, w, h)

    def update(self, dt):
        pass

    def draw(self, screen):
        pygame.draw.rect(screen, cfg.STATIC_OBS, self.rect, border_radius=4)


class MovingObstacle:
    """Slides back and forth between point a and point b."""

    def __init__(self, a, b, size, speed):
        self.a, self.size, self.speed = a, size, speed
        dx, dy = b[0] - a[0], b[1] - a[1]
        self.length = math.hypot(dx, dy)
        self.ux, self.uy = dx / self.length, dy / self.length
        self.t = 0.0
        self.direction = 1

    def update(self, dt):
        self.t += self.direction * self.speed * dt
        if self.t >= self.length:
            self.t, self.direction = self.length, -1
        elif self.t <= 0:
            self.t, self.direction = 0.0, 1

    @property
    def rect(self):
        cx = self.a[0] + self.ux * self.t
        cy = self.a[1] + self.uy * self.t
        return pygame.Rect(cx - self.size / 2, cy - self.size / 2, self.size, self.size)

    def draw(self, screen):
        pygame.draw.rect(screen, cfg.MOVING_OBS, self.rect, border_radius=6)