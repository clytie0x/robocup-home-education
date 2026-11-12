# Task for Claude Code: Help Me Carry (Python)

Read `docs/reference/README.md` first. The full original task is in `docs/reference/original-prompt.md`.

All the Help Me Carry work lives in `hmc/`. The hard rules are in `AGENTS.md`, and the decisions are in `WORKLOG.md`.

## State as of 2026-11-12 21:40 (Asia/Bangkok)

- Slice 1 (615 of 1115) is on `feature/help-me-carry-slice1`, three commits on top of `main`, not pushed.
- Slice 2 is on `feature/help-me-carry-slice2`, which branches from slice 1. It adds the four obstacles, the crowd, the queue and a second map (`--map eindhoven2024`). `pytest` gives `63 passed`. `python -m hmc.run` ends in state `done` with 1115 of 1115 on both maps.
- `docs/evidence/help-me-carry-slice2/` holds the slice 2 outputs, the logs and `SHA256SUMS`.
- Still to do:
  - Push the branches and open PRs to `main`. The user has not approved this yet.
  - Check that CI passes on GitHub. It has not run yet.
