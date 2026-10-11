"""Plan a route from start to goal using only what perception reports."""
import math
import numpy as np
import pygame
from src.utils import config as cfg
from src.simulation.environment import Environment
from src.perception.camera import capture
from src.perception.detector import detect
from src.perception.occupancy_grid import OccupancyGrid
from src.planning.astar import search
from src.planning.smoothing import smooth


def px_length(points):
    return sum(math.hypot(points[i + 1][0] - points[i][0], points[i + 1][1] - points[i][1])
               for i in range(len(points) - 1))


def margin_overlay(grid):
    img = np.zeros((grid.rows, grid.cols, 3), dtype=np.uint8)
    img[grid.inflated == 1] = (160, 50, 50)
    surf = pygame.surfarray.make_surface(np.transpose(img, (1, 0, 2)))
    surf = pygame.transform.scale(surf, (cfg.WIDTH, cfg.HEIGHT))
    surf.set_colorkey((0, 0, 0))
    surf.set_alpha(90)
    return surf


def main():
    pygame.init()
    screen = pygame.display.set_mode((cfg.WIDTH, cfg.HEIGHT))
    pygame.display.set_caption("Path Planning Demo")
    font = pygame.font.SysFont("consolas", 16)
    clock = pygame.time.Clock()

    env = Environment()
    _, mask = detect(capture(env))
    grid = OccupancyGrid(mask)
    start = grid.world_to_cell(*cfg.START_POS)
    goal = grid.world_to_cell(*cfg.GOAL_POS)

    astar = search(grid, start, goal, use_heuristic=True)
    dijk = search(grid, start, goal, use_heuristic=False)

    if astar.path is None:
        print("No path found. Check obstacle layout or INFLATE_PX in config.py")
        return

    smooth_cells = smooth(grid, astar.path)
    raw_px = [grid.cell_to_world(r, c) for r, c in astar.path]
    smooth_px = [grid.cell_to_world(r, c) for r, c in smooth_cells]

    print(f"{'Algorithm':<10}{'Cost(cells)':>13}{'Expanded':>10}{'Time(ms)':>10}")
    print(f"{'A*':<10}{astar.cost:>13.2f}{astar.expanded:>10}{astar.time_ms:>10.1f}")
    print(f"{'Dijkstra':<10}{dijk.cost:>13.2f}{dijk.expanded:>10}{dijk.time_ms:>10.1f}")
    print(f"Raw path length     : {px_length(raw_px):.0f} px ({len(raw_px)} waypoints)")
    print(f"Smoothed path length: {px_length(smooth_px):.0f} px ({len(smooth_px)} waypoints)")

    overlay = margin_overlay(grid)
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_s:
                    pygame.image.save(screen, "outputs/images/03_planning.png")
                    print("Saved outputs/images/03_planning.png")

        env.render(screen, font)
        screen.blit(overlay, (0, 0))
        if len(raw_px) > 1:
            pygame.draw.lines(screen, (255, 220, 80), False, raw_px, 1)
        if len(smooth_px) > 1:
            pygame.draw.lines(screen, (80, 255, 140), False, smooth_px, 3)

        lines = [f"A*       expanded {astar.expanded:5d}  cost {astar.cost:6.1f}",
                 f"Dijkstra expanded {dijk.expanded:5d}  cost {dijk.cost:6.1f}",
                 "yellow = raw A*   green = smoothed",
                 "S: screenshot  Esc: quit"]
        for i, t in enumerate(lines):
            screen.blit(font.render(t, True, cfg.TEXT), (24, 120 + i * 22))

        pygame.display.flip()
        clock.tick(cfg.FPS)

    pygame.quit()


if __name__ == "__main__":
    main()