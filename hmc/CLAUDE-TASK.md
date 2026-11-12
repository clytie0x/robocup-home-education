# Task for Claude Code: Help Me Carry slice 1 (Python)

Read `docs/reference/README.md` first. The full original task is in `docs/reference/original-prompt.md`.

All the slice 1 work now lives in `hmc/`. The hard rules are in `AGENTS.md`, and the decisions are in `WORKLOG.md`.

## State as of 2026-11-12 21:01 (Asia/Bangkok)

- The branch `feature/help-me-carry-slice1` was fast-forwarded to `main` (`ce22168`). Nothing is committed on top of it yet.
- The dependencies are installed in `hmc/.venv`.
- `pytest` gives `49 passed`, and `python -m hmc.run` ends in state `done` with 615 of 1115 at 40.1 s sim time.
- `docs/evidence/help-me-carry-slice1/` holds the outputs, the logs and `SHA256SUMS`.
- `hmc/README.md`, `AGENTS.md` and `WORKLOG.md` are written. The root `README.md` has a row for `hmc/`.
- Still to do:
  - Commit, push the feature branch, and open a PR to `main`.
  - Check that CI passes on GitHub. It has not run yet.
