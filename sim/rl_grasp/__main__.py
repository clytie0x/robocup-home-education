"""python -m rl_grasp train|evaluate"""

import argparse
import time

import numpy as np

from .evaluate import run
from .train import save_checkpoint, train_policy


def main(argv=None):
    parser = argparse.ArgumentParser(prog="rl_grasp")
    sub = parser.add_subparsers(dest="command", required=True)

    train = sub.add_parser("train", help="train a PPO or SAC grasp policy")
    train.add_argument("--algo", choices=("ppo", "sac"), default="ppo")
    train.add_argument("--steps", type=int, required=True, help="environment steps (one grasp try each)")
    train.add_argument("--seed", type=int, default=0)
    train.add_argument("--workers", type=int, default=1, help="CPU processes for the physics")
    train.add_argument("--tries", type=int, default=1, help="tries per episode (PLAN.md tier 1 is one)")
    train.add_argument("--nominal", action="store_true", help="fixed bag, no domain randomisation")
    train.add_argument("--hard", action="store_true", help="wide camera and dynamics ranges")
    train.add_argument("--out", required=True, help="checkpoint path")
    train.add_argument("--hidden", type=int, default=64)

    evaluate = sub.add_parser("evaluate", help="success rate, median time, collisions")
    evaluate.add_argument("--policy", required=True, help="'scripted' or a checkpoint path")
    evaluate.add_argument("--episodes", type=int, required=True)
    evaluate.add_argument("--seed", type=int, default=0)
    evaluate.add_argument("--tries", type=int, default=4)
    evaluate.add_argument("--nominal", action="store_true")
    evaluate.add_argument("--hard", action="store_true")

    args = parser.parse_args(argv)
    if args.command == "train":
        if args.steps < 1:
            raise SystemExit("steps must be at least 1")
        if args.workers < 1:
            raise SystemExit("workers must be at least 1")
        started = time.perf_counter()
        module, returns, times = train_policy(
            args.algo, args.steps, args.seed, args.workers, args.nominal, args.tries, args.hidden,
            hard=args.hard,
        )
        elapsed = time.perf_counter() - started
        save_checkpoint(args.out, args.algo, module, args.steps, args.seed)
        mean = sum(returns) / len(returns) if returns else float("nan")
        instant = sum(1 for value in times if value < 0.2)
        print("wrote %s" % args.out)
        print("episodes: %d" % len(returns))
        print("mean_return: %.4f" % mean)
        print("wall_s: %.3f" % elapsed)
        print("tries_per_s: %.2f" % (len(returns) / elapsed if elapsed else float("nan")))
        if times:
            print("median_time_s: %.3f" % float(np.median(times)))
            print("instant_fraction: %.3f" % (instant / len(times)))
        return 0
    text = run(args.policy, args.episodes, args.seed, args.nominal, args.tries, hard=args.hard)
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
