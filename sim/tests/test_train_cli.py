"""The train command runs a couple of nominal tries and writes a checkpoint."""

from pathlib import Path

from rl_grasp.__main__ import main


def test_two_ppo_steps_write_a_checkpoint(tmp_path):
    out = Path(tmp_path) / "ppo.pt"
    code = main([
        "train", "--algo", "ppo", "--steps", "2", "--seed", "1",
        "--workers", "1", "--tries", "1", "--nominal", "--out", str(out),
    ])
    assert code == 0
    assert out.is_file()
    assert out.stat().st_size > 0
