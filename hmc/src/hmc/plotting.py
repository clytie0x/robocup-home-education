"""Draws trajectory.png. matplotlib is optional, so it is imported lazily."""

from pathlib import Path

from hmc.world import World


def save_trajectory(path: Path, world: World, rows: list[dict]) -> bool:
    """Save the plot. Returns False (and does nothing) if matplotlib is missing."""
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return False

    fig, ax = plt.subplots(figsize=(7, 6))
    ax.plot([r["robot_x"] for r in rows], [r["robot_y"] for r in rows], label="robot")
    ax.plot(
        [r["operator_x"] for r in rows],
        [r["operator_y"] for r in rows],
        "--",
        label="operator",
    )
    ax.plot(*world.start, "gs", markersize=10, label="start")
    ax.plot(*world.car, "k^", markersize=10, label="car (unknown to robot)")
    for bag in world.bags.values():
        ax.plot(bag.x, bag.y, "o", color=bag.name, markersize=7)
        ax.annotate(bag.name, (bag.x, bag.y), textcoords="offset points", xytext=(6, 4))
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Help Me Carry, slice 1 scenario")
    ax.legend(loc="lower right")
    ax.grid(True, alpha=0.3)
    fig.savefig(path, dpi=100)
    plt.close(fig)
    return True
