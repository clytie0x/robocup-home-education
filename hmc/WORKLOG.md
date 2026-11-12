# WORKLOG

Decisions for Help Me Carry slice 1. Dates and times are Asia/Bangkok.

## 2026-11-12

- **Python rewrite of the Rust attempt.** The team is still learning, so slice 1 is plain Python. It reuses the spec and the 615/1115 target from the earlier Rust PR (clytie0x/robocup-home PR #37) as a reference only. The rulebook 2025.1 stays the source of truth.
- **Standard library only.** matplotlib is an optional extra, used only for `trajectory.png`. This keeps the install small and the code easy to read.
- **The car position stays hidden from the task.** `task.py` only knows the start (from odometry) and what perception reports. A test checks the source for `.car`.
- **The robot decides it has arrived when the operator stands still and points.** The car is never told to the robot, so this signal stands in for "we are at the car".
- **Picking is a state flag.** There is no arm model in slice 1.
- **The obstacles, queue, Deus ex Machina and alternative start are not run.** They are listed as `not-run` in the spec, so the scoresheet shows them without claiming them.
- **20:50 Moved everything into `hmc/`.** While this branch was being written, `main` gained `voice/`, and `arm-system` adds `arm/`. One folder per team keeps the root clean. CI runs with `working-directory: hmc`.
- **20:52 Fixed two bugs that the first pytest run found.** The car-position test matched `.carried_bag`, so it now uses the regex `\.car\b`. The run loop also advanced the clock on the step that timed out, so a timeout ended at 300.1 s instead of 300.0 s. Timing out now takes no sim time.
- **21:00 Evidence is stored byte-for-byte.** `core.autocrlf` would rewrite line endings and break `SHA256SUMS`, so `docs/evidence/` has `* -text`. Two runs gave identical hashes for all three outputs.
- **Checked the spec and the code against the rulebook text.** `docs/reference/hmc-2025.txt` was used, and the PDF hash matches `docs/reference/README.md`. All 17 score rows, the -250 Deus ex Machina cap and the 5:00 limit match. There was one wording gap. The spec said `perceive_bag` needs the robot to say the bag, but the referee only checks the perceived bag. The spec now says what the code does.
- **The pytest log is saved with `--no-header`**, so the evidence holds no local machine paths.
- **Added the official score sheet and the restart rule to the spec.** `docs/reference/2025.1_scoresheets.pdf` (Rev 2025-04-28) matches the spec's 17 rows. It adds three tries per test, a "restart after" record and "Total Score 1226" (1115 + 111). The restart rule (rulebook sec. 3.7.9) is in the spec as AT-28. The three tries are AT-29. Both are `not-run`. The rulebook PDF the user supplied has the same hash as `docs/reference/2025.1_rulebook.pdf`.
