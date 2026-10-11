"""Simulated top-down camera. Returns a BGR image like a real OpenCV frame."""
import cv2
import numpy as np
from src.utils import config as cfg


def _bgr(rgb):
    return (rgb[2], rgb[1], rgb[0])


# Fixed noise pattern, generated once so each frame stays cheap.
_NOISE = np.random.default_rng(0).normal(
    0, cfg.CAMERA_NOISE, (cfg.HEIGHT, cfg.WIDTH, 3)).astype(np.int16)


def _fill(frame, rect, rgb):
    cv2.rectangle(frame, (rect.left, rect.top), (rect.right - 1, rect.bottom - 1),
                  _bgr(rgb), -1)


def capture(env):
    """Render the world (without the car or HUD) as the camera would see it."""
    frame = np.full((cfg.HEIGHT, cfg.WIDTH, 3), _bgr(cfg.BG), dtype=np.uint8)
    for w in env.walls:
        _fill(frame, w, cfg.WALL)
    for o in env.static_obstacles:
        _fill(frame, o.rect, cfg.STATIC_OBS)
    for o in env.moving_obstacles:
        _fill(frame, o.rect, cfg.MOVING_OBS)
    if cfg.CAMERA_NOISE > 0:
        frame = np.clip(frame.astype(np.int16) + _NOISE, 0, 255).astype(np.uint8)
    return frame