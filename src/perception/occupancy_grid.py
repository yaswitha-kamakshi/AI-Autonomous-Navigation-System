"""Occupancy grid: 0 = free, 1 = blocked. Obstacles are inflated by a safety margin."""
import math
import cv2
import numpy as np
from src.utils import config as cfg


class OccupancyGrid:
    def __init__(self, mask):
        self.rows = cfg.HEIGHT // cfg.CELL
        self.cols = cfg.WIDTH // cfg.CELL
        # A cell is blocked if ANY pixel inside it is an obstacle.
        blocks = (mask > 0).reshape(self.rows, cfg.CELL, self.cols, cfg.CELL)
        self.raw = blocks.max(axis=(1, 3)).astype(np.uint8)

        # Inflate so the planner keeps the car's body away from obstacles.
        r = math.ceil(cfg.INFLATE_PX / cfg.CELL)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))
        self.inflated = cv2.dilate(self.raw, kernel)

    def world_to_cell(self, x, y):
        """World pixels (x, y) -> grid (row, col)."""
        return int(y // cfg.CELL), int(x // cfg.CELL)

    def cell_to_world(self, row, col):
        """Grid (row, col) -> world pixels at the cell center."""
        return (col + 0.5) * cfg.CELL, (row + 0.5) * cfg.CELL

    def in_bounds(self, row, col):
        return 0 <= row < self.rows and 0 <= col < self.cols

    def is_free(self, row, col):
        return self.in_bounds(row, col) and self.inflated[row, col] == 0