"""Drive the car with the keyboard to test the simulation."""
import pygame
from src.utils import config as cfg
from src.simulation.environment import Environment


def main():
    pygame.init()
    screen = pygame.display.set_mode((cfg.WIDTH, cfg.HEIGHT))
    pygame.display.set_caption("AI Autonomous Navigation - Manual Test")
    font = pygame.font.SysFont("consolas", 18)
    clock = pygame.time.Clock()
    env = Environment()

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

        keys = pygame.key.get_pressed()
        throttle = (1 if keys[pygame.K_UP] else 0) - (1 if keys[pygame.K_DOWN] else 0)
        steer = (1 if keys[pygame.K_RIGHT] else 0) - (1 if keys[pygame.K_LEFT] else 0)

        env.step(throttle, steer * cfg.MAX_STEER)
        env.render(screen, font)
        pygame.display.flip()
        clock.tick(cfg.FPS)

    pygame.quit()


if __name__ == "__main__":
    main()