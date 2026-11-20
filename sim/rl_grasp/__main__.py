"""python -m rl_grasp train|evaluate"""

import argparse

from .evaluate import run
from .train import save_checkpoint, train_policy


def main(argv=None):
    parser = argparse.ArgumentParser(prog="rl_grasp")
    sub = parser.add_subparsers(dest="command", required=True)

    train = sub.add_parser("train", help="train a PPO or SAC grasp policy")
    train.add_argument("--algo", choices=("ppo", "sac"), default="ppo")
    train.add_argument("--steps", type=int, required=True, help="environment steps (one grasp try each)")
    train.add_argument("--seed", type=int, default=0)
    train.add_argument("--workers", type=int, default=1, help="CPU processes. PPO only; SAC updates in-process")
    train.add_argument("--tries", type=int, default=1, help="tries per episode (PLAN.md tier 1 is one)")
    train.add_argument("--nominal", action="store_true", help="fixed bag, no domain randomisation")
    train.add_argument("--out", required=True, help="checkpoint path")
    train.add_argument("--hidden", type=int, default=64)

    evaluate = sub.add_parser("evaluate", help="success rate, median time, collisions")
    evaluate.add_argument("--policy", required=True, help="'scripted' or a checkpoint path")
    evaluate.add_argument("--episodes", type=int, required=True)
    evaluate.add_argument("--seed", type=int, default=0)
    evaluate.add_argument("--tries", type=int, default=4)
    evaluate.add_argument("--nominal", action="store_true")

    args = parser.parse_args(argv)
    if args.command == "train":
        if args.steps < 1:
            raise SystemExit("steps must be at least 1")
        if args.workers < 1:
            raise SystemExit("workers must be at least 1")
        module, returns = train_policy(
            args.algo, args.steps, args.seed, args.workers, args.nominal, args.tries, args.hidden,
        )
        save_checkpoint(args.out, args.algo, module, args.steps, args.seed)
        mean = sum(returns) / len(returns) if returns else float("nan")
        print("wrote %s" % args.out)
        print("episodes: %d" % len(returns))
        print("mean_return: %.4f" % mean)
        return 0
    text = run(args.policy, args.episodes, args.seed, args.nominal, args.tries)
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
