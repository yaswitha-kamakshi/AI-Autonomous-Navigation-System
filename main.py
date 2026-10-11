"""Fully autonomous run: perceive -> plan -> control, with a live dashboard.

Keys:  B = drop a surprise obstacle on the route   R = restart
       S = save screenshot                          Esc = quit
Run with --record to save outputs/videos/demo.mp4 (the run ends by itself).
"""
import argparse
import math
import numpy as np
import pygame
from src.utils import config as cfg
from src.simulation.environment import Environment
from src.navigation import AutonomousAgent, State, drop_block_on_path

PANEL_W = 344
BOX_COLORS = {"static": (255, 120, 120), "moving": (255, 210, 90)}
STATE_COLORS = {
    State.NAVIGATING: (90, 220, 130), State.AVOIDING: (255, 180, 60),
    State.RECOVERING: (255, 130, 60), State.REPLANNING: (120, 180, 255),
    State.GOAL_REACHED: (90, 255, 160), State.FAILED: (255, 90, 90),
    State.IDLE: (200, 200, 200),
}


def grid_surface(grid, scale=3):
    img = np.zeros((grid.rows, grid.cols, 3), dtype=np.uint8)
    img[:] = (20, 22, 28)
    img[grid.inflated == 1] = (120, 50, 50)
    img[grid.raw == 1] = (230, 230, 230)
    surf = pygame.surfarray.make_surface(np.transpose(img, (1, 0, 2)))
    return pygame.transform.scale(surf, (grid.cols * scale, grid.rows * scale))


def draw_frame(screen, env, agent, font, big_font):
    env.render(screen, font)

    # Planned path (only the part still ahead of the car)
    ahead = agent.controller.path[agent.controller.idx:]
    if len(ahead) > 1:
        pygame.draw.lines(screen, (80, 255, 140), False, ahead, 2)

    # Detections from the OpenCV pipeline
    for d in agent.detections:
        if d.label in BOX_COLORS:
            pygame.draw.rect(screen, BOX_COLORS[d.label],
                             (d.x - 2, d.y - 2, d.w + 4, d.h + 4), 1)

    # Range-sensor beams (red = close, green = far)
    for ang, dist in agent.readings:
        end = (env.car.x + math.cos(ang) * dist, env.car.y + math.sin(ang) * dist)
        frac = dist / cfg.RAY_MAX
        pygame.draw.line(screen, (int(255 * (1 - frac)), int(255 * frac), 60),
                         (env.car.x, env.car.y), end, 1)

    # Side panel
    px = cfg.WIDTH
    pygame.draw.rect(screen, (24, 26, 32), (px, 0, PANEL_W, cfg.HEIGHT))
    pygame.draw.line(screen, cfg.WALL, (px, 0), (px, cfg.HEIGHT), 2)
    screen.blit(font.render("Occupancy grid (what the planner sees)", True, cfg.TEXT), (px + 10, 10))
    if agent.grid is not None:
        screen.blit(grid_surface(agent.grid), (px + 10, 34))

    color = STATE_COLORS.get(agent.state, cfg.TEXT)
    screen.blit(big_font.render(agent.state.value, True, color), (px + 10, 268))
    nearest = agent.status.nearest if agent.status else cfg.RAY_MAX
    objects = sum(d.label != "wall" for d in agent.detections)
    lines = [f"Objects detected : {objects}",
             f"Nearest (front)  : {nearest:6.1f}",
             f"Speed / target   : {env.car.speed:5.1f} / {agent.target_speed:5.1f}",
             f"Distance to goal : {env.distance_to_goal():6.1f}",
             f"Replans          : {agent.replans}",
             f"Recoveries       : {agent.recoveries}",
             f"Collisions       : {env.collisions}",
             f"Time             : {env.time:6.1f}s",
             "",
             "green line = planned path",
             "B: drop obstacle  R: restart",
             "S: screenshot     Esc: quit"]
    for i, text in enumerate(lines):
        screen.blit(font.render(text, True, cfg.TEXT), (px + 10, 305 + i * 22))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--record", action="store_true", help="save outputs/videos/demo.mp4")
    args = parser.parse_args()

    pygame.init()
    screen = pygame.display.set_mode((cfg.WIDTH + PANEL_W, cfg.HEIGHT))
    pygame.display.set_caption("AI-Based Autonomous Navigation System")
    font = pygame.font.SysFont("consolas", 16)
    big_font = pygame.font.SysFont("consolas", 28, bold=True)
    clock = pygame.time.Clock()

    env = Environment()
    agent = AutonomousAgent(env)

    writer = None
    if args.record:
        import imageio.v2 as imageio
        writer = imageio.get_writer("outputs/videos/demo.mp4", fps=15)

    frame_no, done_frames, last_state = 0, 0, None
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
                    agent = AutonomousAgent(env)
                    done_frames, last_state = 0, None
                elif event.key == pygame.K_b:
                    if drop_block_on_path(env, agent):
                        print("Dropped a surprise obstacle on the route")
                elif event.key == pygame.K_s:
                    pygame.image.save(screen, "outputs/images/04_autonomous_run.png")
                    print("Saved outputs/images/04_autonomous_run.png")

        throttle, steer = agent.step()
        env.step(throttle, steer)
        draw_frame(screen, env, agent, font, big_font)

        if agent.state != last_state:
            print(f"t={env.time:5.1f}s  state -> {agent.state.value}")
            last_state = agent.state

        if writer is not None and frame_no % 2 == 0:
            writer.append_data(np.transpose(pygame.surfarray.array3d(screen), (1, 0, 2)))

        if agent.state in (State.GOAL_REACHED, State.FAILED):
            done_frames += 1
            if args.record and done_frames > 45:
                running = False

        pygame.display.flip()
        clock.tick(cfg.FPS)
        frame_no += 1

    if writer is not None:
        writer.close()
        print("Saved outputs/videos/demo.mp4")
    print(f"RESULT: {agent.state.value} | time={env.time:.1f}s | collisions={env.collisions} "
          f"| replans={agent.replans} | recoveries={agent.recoveries}")
    pygame.quit()


if __name__ == "__main__":
    main()