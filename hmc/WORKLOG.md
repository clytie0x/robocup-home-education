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

## 2026-11-12, slice 2 (branch `feature/help-me-carry-slice2`)

- **21:00 Goal: earn all 1115 points in the scripted run.** The four obstacles (4 x 50) and the queue (300) were the missing rows. Slice 2 branches from slice 1, so the slice 1 PR stays small.
- **Obstacles are shapes, and the robot does not know their kind.** Perception returns circles and boxes only. The hard-to-see object is only seen within 1.5 m. This keeps the robot code honest: it cannot cheat by reading `world.obstacles`, and a test checks this.
- **A* on a 0.2 m grid, built from what the robot sees now.** The rulebook says the area is unmapped, so there is no stored map. The grid covers the robot, the goal and 3 m around them. Cells closer to a shape than 0.41 m (robot radius 0.26 m plus 0.15 m) are blocked. The plan is redone every 0.5 s, when the goal moves more than 0.3 m, or when a new shape appears. A* is short, needs only the standard library, and a beginner can follow it step by step.
- **The robot radius is 0.26 m.** It comes from the real base, a 0.40 m square with 0.04 m corners cut off, as recorded in the team's hardware handoff.
- **The referee judges "avoid" rows on arrival at the car.** A row is awarded when the robot came within 1.0 m of the obstacle and never got closer than its own radius. The first try used 2.0 m. A test found that this was too loose: the car is 1.4 m from the barriers, so a robot placed at the car "faced" them without passing them.
- **The queue tail is not "the person farthest from the start".** The first try used that rule, and it broke because the queue walks on past the start. The referee now uses the last person to start walking. The robot uses the closest walking person who is ahead of it.
- **Speed stays at 1.0 m/s in the simulation.** The real base is limited to 0.15 m/s. At that speed the robot cannot follow a person walking at 0.8 m/s, so this is noted as a limit and not hidden.
- **21:50 Added a second map, `eindhoven2024`.** No official map exists for Help Me Carry 2025 (the Salvador 2025 repo leaves the section empty, because the test runs outside the arena). The Carry My Luggage maps of 2022 to 2025 show only starting points and never the car. The Eindhoven 2024 map draws the whole venue (team area, arenas, audience stands), so it is the closest to "a public space outside the arena". We copied its boxes at 5 cm per pixel, because the image has no scale. The car, the obstacles and the crowd are placed by us. The default map stays as it is, so the old tests did not change.
- **The plot is now 9 x 7 inches**, so the larger map stays readable. This changed the bytes of `trajectory.png`, so the slice 2 evidence was regenerated.
