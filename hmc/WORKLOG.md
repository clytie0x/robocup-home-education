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
- **22:10 Maps for every official Carry My Luggage map.** The RoboCupAtHome GitHub has Carry My Luggage maps for Bangkok 2022, Bordeaux 2023, Eindhoven 2024, German Open 2024, Portugal Open 2024 and German Open 2025. The last one is a placeholder image, so it is left out. In three of them the robot starts inside the flat and has to leave through the exit door first. None of them shows the outside, so we put the default course outside the exit, turned away from the door. Only German Open 2024 gives a scale (10 m), so the other flats are also taken as 10 m wide. The walls are traced by eye.
- **Walls now block the view.** Without this, the robot could "see" the operator through a wall, and leaving a room would be too easy.
- **Grid 0.1 m and safety 0.10 m (was 0.2 m and 0.15 m).** With the old values the robot could not fit through a 1 m door. The smaller safety made the robot cut a corner and get stuck next to a table on the Eindhoven map. Two changes fixed it: the robot now turns on the spot when it faces more than 30 degrees off its path (a two-wheel base can do that), and the planner may move the robot out of the safety zone, but never closer to anything.
- **Queue fixes found by the new maps.** On Eindhoven with the green bag, a corner hid the last person and the robot gave up the queue. It now keeps going for 3 s to where it last saw them. It also stands behind the last person in their walking direction, not just 1 m away in any direction, because that put it next to the queue. The queue row also ends when the last person has walked to the end of their route, because on Portugal Open the last person stops 2.4 m from the start. On Eindhoven the queue route now passes the start on the left, because a scripted person walked into the robot standing at the start.
- **23:00 Passers-by, sensor noise and re-ID.** The rulebook allows anybody to walk by during the whole test, so `--bystanders N` adds random passers-by who step away from the robot. `--noise realistic|hard` adds the mistakes a small YOLO detector and a simple tracker make: position error, misses, pointing error, tracker id swaps, and frames where someone else is taken for the operator. The levels are our guess. `python -m hmc.robustness` runs every map with many seeds and counts the runs that end in DONE with 1115 points and no touch.
- **Rules beat guesses for the queue.** The robot first took "anyone walking ahead of us" for the queue. With passers-by that fell to about 40 % success, and more hand-made rules (walking direction, walking in pairs, locking onto one person) made it worse. The rulebook says "The queue is composed of the same people that form the crowd." Remembering the people who stood together in a group on the way out, and only following them back, brought it to 95 % without noise.
- **Noise needed averaging, gating and voting.** With noise and no changes the robot never finished (0 %): a noisy operator never looked still, so the robot never noticed it had arrived. Positions are now averaged over 0.5 s, detections that jump further than a person can walk are ignored, the bag needs 5 votes, and a person missed for up to 1 s is still avoided. The operator is now tracked in every state; before, the robot stopped avoiding the operator after following and could drive through them.
- **Re-ID by look.** Tracker id swaps broke "remember the crowd by id". Every person now has a random look (a 16-number vector, like a re-ID embedding) that stays with them when ids swap. The robot learns the operator's look at the start and the crowd's looks on the way out, and matches by look. With 10 passers-by and realistic noise this raised success from 50 % to 84 % (`docs/evidence/help-me-carry-slice2/robustness/`). With hard noise it is 36 %, mostly the queue, and touches start to appear.

## 2026-11-13, realistic speeds and pace (branch `claude/hmc-speed-cf7458`, from `feature/help-me-carry-slice2`)

- **01:10 Goal: realistic speeds, and let the robot manage the operator's pace within the rules.** Before, the robot drove 1.0 m/s with instant acceleration and the operator walked 0.8 m/s. The real base is capped at 0.15 m/s and 15 deg/s today, and people walk about 1.2 to 1.4 m/s.
- **Acceleration limit.** The base speeds up and brakes at up to 0.5 m/s^2. Its turn rate (still up to 2 rad/s) changes by up to 3 rad/s^2: the same 0.5 m/s^2 per wheel on a two-wheel base about 0.34 m wide, turning on the spot. `--robot-speed` (both CLIs) is the top speed. The turn rate cap is not tied to `--robot-speed`, so a 0.15 m/s run still turns at up to 2 rad/s, not the real 15 deg/s.
- **The limit broke the navigator, and these changes fixed it.** With the limit alone the default run lost the queue (815). What we found and changed:
  - The robot wobbled (heading swinging by up to 1 rad), because it aimed at the next grid cell, 0.15 m ahead, and could no longer turn at once. It now aims at a waypoint up to 0.4 m ahead. The heading also stops at its target instead of swinging past it.
  - A robot that cannot stop at once rolls on when the plan changes. It now brakes in time for the end of its path and for anything straight ahead, where it faces and where it is turning to. It only counts things that rolling on brings it closer to, otherwise it got stuck next to a wall. An earlier try that slowed down near anything, in any direction, made it stop for good next to obstacles (61 % on a quick try), so it was dropped.
  - Turning while still rolling fast swept the robot sideways into the barriers. When the path turns back more than 90 degrees, it now brakes to a stop first, then turns on the spot. Doing this already at 30 degrees made it stop all the time and lose the queue.
  - Aiming 0.4 m ahead popped the last waypoint too, so the robot stopped short of the bag for good. The last waypoint is now kept. When the straight line to the far waypoint cuts too close to something, it aims at the next one.
  - Braking from sqrt(2 a d) was one step late, so the robot reached the end of its path still at 0.25 m/s and stopped at once. It now brakes one step earlier.
- **People.** The operator's natural speed is 1.2 m/s (`--operator-speed`), passers-by walk 1.0 to 1.4 m/s (was 0.6 to 1.2). The queue still walks 0.5 m/s.
- **The robot asks the operator to walk slowly.** The rulebook says the referees tell the operator to "follow the robots instructions". Right after "I am ready to follow you." the robot says "Please walk slowly, at about X metres per second, so I can follow you." with X = 0.8 x its top speed. The operator walks at X, or slower if that is their own pace. The sentence comes after "ready", so the first sentence is still the ready one.
- **The robot asks the operator to wait.** When it sees it is more than 4 m behind and it has a path, it says "Please wait for me." and the operator stands still. Below 2.5 m it says "Thank you, please go on." The first version also waited when the robot had no path, and on Portugal Open the operator then waited in the door the robot needed, forever. Now no path also means "please go on". In the default run the robot asks twice: going around the small object, and going around the crowd.
- **Test changed.** `test_falling_back_from_the_queue_loses_the_queue_row` stopped the robot at a fixed time (40 to 46 s). With the slower robot that time now falls before the queue, so the robot still earned it. The test now stops the robot for 6 s right after it got within 2 m of the last person of the queue. It checks the same thing as before.
- **Results.** `pytest`: 105 passed. `python -m hmc.run` (default map): 1115 of 1115, done in 65.6 s (was 52.2 s). Scripted runs by top speed:

  | Top speed | default map | eindhoven2024 |
  |---|---|---|
  | 1.0 m/s | 1115, done, 65.6 s | 1115, done, 133.1 s |
  | 0.5 m/s | 815 (no queue), done, 87.9 s | 815 (no queue), done, 225.1 s |
  | 0.3 m/s | 815 (no queue), done, 142.2 s | 615, timeout |
  | 0.15 m/s | 815 (no queue), done, 277.8 s | 0, timeout |

  `python -m hmc.robustness --seeds 10` (10 passers-by), runs that end in DONE with 1115 and no touch, out of 60:

  | Top speed | noise off, ideal sensors | `--sensors robot --noise realistic` |
  |---|---|---|
  | before this change (1.0 m/s, no limit) | 56 (93.3 %) | 27 (45.0 %) |
  | 1.0 m/s | 51 (85.0 %) | 28 (46.7 %) |
  | 0.5 m/s | 40 (66.7 %) | 40 (66.7 %) |
  | 0.3 m/s | 40 (66.7 %) | 40 (66.7 %) |
  | 0.15 m/s | 7 (11.7 %) | 6 (10.0 %) |

  At 0.5 and 0.3 m/s Bangkok, Bordeaux, German Open and Portugal Open are 10/10 with both sensor models, while `default` and `eindhoven2024` are 0/10: there the robot cannot catch the queue (0.5 m/s), and at 0.3 m/s Eindhoven also times out. At 1.0 m/s with robot sensors there are touches, mostly the barriers at 0.00 m: the robot sees them late and needs 1 m to stop from 1 m/s.
- **0.15 m/s cannot finish within 5:00 on most maps.** 48 of 60 runs (ideal sensors) and 47 of 60 (robot sensors) end in timeout. Only Bordeaux finishes some runs (7 and 6 of 10). The default map finishes the scripted run in 277.8 s but loses the queue. The robot can never join the queue at 0.15 m/s, because the queue walks 0.5 m/s and the robot may only ask the operator, not the queue, to slow down.
- **Not done here.** The slice 2 evidence folder (`docs/evidence/help-me-carry-slice2/`) is not regenerated, because two other sessions change the behaviour at the same time (odometry drift, lost-operator recovery). Regenerate it once the branches are merged.
