"""Run many randomized scenarios headlessly and report success/collision metrics.

Usage:  python evaluate.py            (50 runs)
        python evaluate.py --runs 100
"""
import argparse
import csv
import os
import random
import statistics
import sys

# Run without opening a window (must be set before pygame is imported)
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

from src.simulation.environment import Environment
from src.navigation import AutonomousAgent, State, drop_block_on_path
from src.utils import config as cfg


def randomize(env, rng):
    """Vary the scenario: obstacle positions, moving-obstacle phase, direction."""
    for o in env.static_obstacles:
        o.rect.x += rng.randint(-12, 12)
        o.rect.y += rng.randint(-12, 12)
    for o in env.moving_obstacles:
        o.t = rng.uniform(0, o.length)
        o.direction = rng.choice([-1, 1])


def run_one(seed):
    rng = random.Random(seed)
    env = Environment()
    randomize(env, rng)
    agent = AutonomousAgent(env)
    drop_time = rng.uniform(1.0, 8.0) if rng.random() < 0.5 else None   # surprise block in 50% of runs
    dropped = False
    path_len, last = 0.0, (env.car.x, env.car.y)

    while agent.state not in (State.GOAL_REACHED, State.FAILED):
        throttle, steer = agent.step()
        env.step(throttle, steer)
        path_len += ((env.car.x - last[0]) ** 2 + (env.car.y - last[1]) ** 2) ** 0.5
        last = (env.car.x, env.car.y)
        if drop_time is not None and not dropped and env.time >= drop_time:
            dropped = drop_block_on_path(env, agent)

    return {
        "seed": seed,
        "success": int(agent.state == State.GOAL_REACHED),
        "collisions": env.collisions,
        "time_s": round(env.time, 2),
        "distance_px": round(path_len, 1),
        "replans": agent.replans,
        "recoveries": agent.recoveries,
        "surprise_block": int(dropped),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--runs", type=int, default=50)
    args = parser.parse_args()

    os.makedirs("outputs/logs", exist_ok=True)
    rows = []
    for i in range(args.runs):
        row = run_one(seed=i)
        rows.append(row)
        print(f"run {i + 1:3d}/{args.runs}  success={row['success']}  "
              f"collisions={row['collisions']}  time={row['time_s']:5.1f}s  replans={row['replans']}")

    with open("outputs/logs/metrics.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)

    n = len(rows)
    ok = [r for r in rows if r["success"]]
    clean = [r for r in ok if r["collisions"] == 0]
    print("\n========== SUMMARY ==========")
    print(f"Runs                      : {n}")
    print(f"Goal reached              : {len(ok)}/{n} ({100 * len(ok) / n:.0f}%)")
    print(f"Collision-free successes  : {len(clean)}/{n} ({100 * len(clean) / n:.0f}%)")
    print(f"Avg collisions per run    : {statistics.mean(r['collisions'] for r in rows):.2f}")
    if ok:
        print(f"Avg time to goal          : {statistics.mean(r['time_s'] for r in ok):.1f} s")
        print(f"Avg distance travelled    : {statistics.mean(r['distance_px'] for r in ok):.0f} px")
    print(f"Avg replans per run       : {statistics.mean(r['replans'] for r in rows):.1f}")
    print("Saved outputs/logs/metrics.csv")

    try:
        make_charts(rows)
    except ImportError:
        print("matplotlib not found, skipping charts")


def make_charts(rows):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    times = [r["time_s"] for r in rows if r["success"]]
    coll = [r["collisions"] for r in rows]
    replans = [r["replans"] for r in rows]
    n_ok = sum(r["success"] for r in rows)
    n_clean = sum(1 for r in rows if r["success"] and r["collisions"] == 0)

    fig, ax = plt.subplots(1, 4, figsize=(18, 4))
    ax[0].bar(["Goal reached", "Collision-free", "Failed"],
              [n_ok, n_clean, len(rows) - n_ok], color=["#3cb371", "#4682b4", "#cd5c5c"])
    ax[0].set_title("Outcomes"); ax[0].set_ylabel("runs")
    ax[1].hist(times, bins=10, color="#4682b4"); ax[1].set_title("Time to goal (s)")
    ax[2].hist(coll, bins=range(0, max(coll) + 2), color="#cd853f", align="left")
    ax[2].set_title("Collisions per run")
    ax[3].hist(replans, bins=range(0, max(replans) + 2), color="#9370db", align="left")
    ax[3].set_title("Replans per run")
    fig.suptitle(f"Evaluation over {len(rows)} randomized scenarios")
    fig.tight_layout()
    os.makedirs("outputs/images", exist_ok=True)
    fig.savefig("outputs/images/05_evaluation.png", dpi=110)
    print("Saved outputs/images/05_evaluation.png")


if __name__ == "__main__":
    sys.exit(main())