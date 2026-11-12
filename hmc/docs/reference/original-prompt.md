You are working in the git repo C:\Users\teera\dev\robocup-home-education. It is currently a fresh repo whose only commit is "Initial commit" with README.md.
Your task is to build slice 1 of RoboCup@Home rulebook 2025.1 section 5.1 Help Me Carry, in Python.

Inputs, outside the repo, read-only:
- C:\Users\teera\dev\hmc-inputs\2025.1_rulebook.pdf is the main @Home rulebook 2025.1, Final, Rev 2025-04-28. Use section 5.1 on pp. 39-41.
- C:\Users\teera\dev\hmc-inputs\hmc-2025.txt is a text extract of section 5.1. Read it yourself and treat the rulebook as the source of truth.

Hard rules:
- Python only. No Rust. All repo content in English.
- The code must be readable and beginner-friendly, because the team is still learning.
- Simulation only. No hardware, deploy, flash, SSH, serial, USB or GPIO code.
- Install packages only inside a repo-local .venv, using pip. Change nothing outside this repo and hmc-inputs.
- Do not delete files or branches, do not force-push, and do not change repo visibility. Do not push to main and do not merge.

Deliverables:
1. docs/tasks/5.1-help-me-carry.md: the exact spec of setup, procedure, rules, the 5:00 time limit, and the full scoresheet. The scoresheet rows are:
   - follow to car 300
   - perceive correct bag 15
   - pick correct bag 100
   - avoid crowd, small object, hard-to-see object and retractable barriers, 50 each
   - re-reach start 200
   - queue 300
   - drop bag -50
   - Deus ex Machina -50/-100/-150/-200
   - not attending -500
   - alternative start signal -100
   - outstanding performance 111
   - total 1115
   Add acceptance tests, each with a status: implemented, not-run, or out-of-sim-scope.
2. A plain-Python package (src layout, e.g. src/hmc/) with small modules: world (a 2D kinematic sim), perception, navigation (follow and go-to), manipulation, hri, task (state machine) and scoresheet. Depend on the standard library only; matplotlib is an optional extra for the plot. Keep interfaces ROS-free so each module can later be wrapped as a ROS 2 Jazzy node on Ubuntu 24.04. Don't add ROS now.
3. A deterministic scripted scenario CLI (python -m hmc.run). It must:
   - start the robot at the start point;
   - follow a scripted operator to a car point the robot is not told in advance;
   - choose which of at least 2 bags the operator points at, via a scripted pointing ray;
   - "pick" the bag as an abstract state flag, with no real grasp;
   - return to the start;
   - respect the 5:00 sim budget.
   Obstacles, the queue, Deus ex Machina and the alternative start stay not-run. The CLI writes a score table, timeline.csv and trajectory.png to an output folder.
4. pytest tests for:
   - every scoresheet row and the total;
   - the state machine transitions;
   - following;
   - bag selection;
   - the return;
   - a timeout at 5:00;
   - the drop penalty;
   - the end-to-end scenario.
5. .github/workflows/ci.yml that runs pytest and the scenario on ubuntu-latest and uploads the outputs.
6. Replace README.md with a short beginner-friendly guide covering:
   - what this is;
   - the Windows PowerShell steps (py -m venv .venv, .venv\Scripts\Activate.ps1, pip install -e .[dev], pytest, python -m hmc.run);
   - the Ubuntu steps;
   - the folder map;
   - the limits.
   Also add AGENTS.md (rules for coding agents, including the hard rules above) and WORKLOG.md (decisions, with dates in Asia/Bangkok).

Process:
- Create branch feature/help-me-carry-slice1.
- Keep the code simple and small.
- Actually run pytest and the scenario locally. Fix until everything passes.
- Commit the generated outputs under docs/evidence/help-me-carry-slice1/ with a SHA256SUMS file.
- Commit, push the branch, and open a PR to main with gh pr create. The PR body must give the real test count and run output.

Final message: give the PR URL, the exact pytest summary line, the scenario score table (awarded total out of 1115, plus sim time), the evidence paths with their SHA256, and the limits. Claim only what you actually ran.
