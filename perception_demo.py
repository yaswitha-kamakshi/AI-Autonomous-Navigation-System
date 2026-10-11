"""Drive manually and watch what the perception system sees."""
import math
import numpy as np
import pygame
from src.utils import config as cfg
from src.simulation.environment import Environment
from src.perception.camera import capture
from src.perception.detector import detect
from src.perception.occupancy_grid import OccupancyGrid
from src.perception.range_sensor import scan

PANEL_W = 320
BOX_COLORS = {"static": (255, 120, 120), "moving": (255, 210, 90)}


def grid_surface(grid, scale=3):
    img = np.zeros((grid.rows, grid.cols, 3), dtype=np.uint8)
    img[:] = (20, 22, 28)
    img[grid.inflated == 1] = (120, 50, 50)      # safety margin
    img[grid.raw == 1] = (230, 230, 230)         # real obstacle cells
    surf = pygame.surfarray.make_surface(np.transpose(img, (1, 0, 2)))
    return pygame.transform.scale(surf, (grid.cols * scale, grid.rows * scale))


def main():
    pygame.init()
    screen = pygame.display.set_mode((cfg.WIDTH + PANEL_W, cfg.HEIGHT))
    pygame.display.set_caption("Perception Demo")
    font = pygame.font.SysFont("consolas", 16)
    clock = pygame.time.Clock()
    env = Environment()
    frame_count = 0

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_r:
                    env.reset()
                elif event.key == pygame.K_s:
                    pygame.image.save(screen, "outputs/images/02_perception.png")
                    print("Saved outputs/images/02_perception.png")

        keys = pygame.key.get_pressed()
        throttle = (1 if keys[pygame.K_UP] else 0) - (1 if keys[pygame.K_DOWN] else 0)
        steer = (1 if keys[pygame.K_RIGHT] else 0) - (1 if keys[pygame.K_LEFT] else 0)
        env.step(throttle, steer * cfg.MAX_STEER)

        # ---- Perception pipeline ----
        frame = capture(env)
        detections, mask = detect(frame)
        grid = OccupancyGrid(mask)
        readings = scan(env)

        # ---- Draw ----
        env.render(screen, font)
        for d in detections:
            if d.label in BOX_COLORS:
                pygame.draw.rect(screen, BOX_COLORS[d.label], (d.x - 2, d.y - 2, d.w + 4, d.h + 4), 2)
                screen.blit(font.render(d.label, True, BOX_COLORS[d.label]), (d.x, d.y - 18))
        for ang, dist in readings:
            end = (env.car.x + math.cos(ang) * dist, env.car.y + math.sin(ang) * dist)
            frac = dist / cfg.RAY_MAX
            color = (int(255 * (1 - frac)), int(255 * frac), 60)   # red = close, green = far
            pygame.draw.line(screen, color, (env.car.x, env.car.y), end, 1)

        px = cfg.WIDTH
        pygame.draw.line(screen, cfg.WALL, (px, 0), (px, cfg.HEIGHT), 2)
        screen.blit(font.render("Occupancy grid", True, cfg.TEXT), (px + 10, 12))
        screen.blit(grid_surface(grid), (px + 10, 36))
        nearest = min(d for _, d in readings)
        info = [f"Objects detected: {sum(d.label != 'wall' for d in detections)}",
                f"Nearest range   : {nearest:5.1f}",
                "",
                "white  = obstacle",
                "red    = safety margin",
                "",
                "S: screenshot  R: reset",
                "Arrows: drive"]
        for i, text in enumerate(info):
            screen.blit(font.render(text, True, cfg.TEXT), (px + 10, 270 + i * 22))

        pygame.display.flip()
        clock.tick(cfg.FPS)
        frame_count += 1
        if frame_count % 30 == 0:
            print(f"objects={sum(d.label != 'wall' for d in detections)}  nearest_range={nearest:.1f}")

    pygame.quit()


if __name__ == "__main__":
    main()