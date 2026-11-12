# The way back for Help Me Carry: design

Status: approved by the owner on 2026-11-13. Requested by the owner as "the way back for the
real robot: back to the start without a map, and following the queue".

Builds on `2026-11-13-person-follow-design.md` and its plan. Nothing here starts before
that plan's Tasks 0 to 12 are done, because it reuses their brain, body, senses, safety
stop and control loop.

## What the rules ask

From `hmc/docs/tasks/5.1-help-me-carry.md`, which restates rulebook 2025.1 section 5.1:

- Main goal 4, bag delivery: the robot takes the bag to the starting location.
- Bonus, 200 points: re-reaching the starting location.
- Bonus, 300 points: joining and staying in the queue on the way back. The queue is a
  few people from the crowd the robot passed on the way out. They walk back to the start
  in a line, and the robot may join in a way that looks natural to them.
- The area outside the arena is not mapped in advance.

## Goals

1. From wherever following ended, drive back and stop within 0.5 m of where the run
   started (the simulation's referee accepts 1 m; 0.5 m leaves room for a strict one).
2. On the way, when people of the crowd walk back in a line ahead of the robot, join
   behind the last one and stay there until 2 m from the start.
3. Touch nothing and nobody, as in following.
4. The same decision code in the simulation and on the robot.

## Scope

In: the drive back and the queue.

Out: picking up and holding the bag (arm work), and the voice command that starts the
way back. Until those exist, the owner presses a key on the notebook to say "we are at
the car, go back", and the robot carries nothing. The hooks for both are named below.

Failures even if everything else works:
- touching a person or an object;
- stopping somewhere else and reporting that it is the start;
- following a passer-by as if they were the queue.

## Why not just drive toward the start

The simulation does exactly that today: it remembers the start in its odometry frame and
plans straight to it with A* from what it sees. That has two weaknesses on the real robot.

1. **Drift.** The simulation's own result: with wheel odometry of 2 % distance error the
   robot missed the start after the long routes (15 of 18 runs), and the WORKLOG concludes
   that the real robot needs SLAM "or something like it". The real base is calibrated to
   about 1 % in distance and 0.3 % in turns over short test moves; nobody has measured it
   over 30 m with a dozen turns.
2. **Dead ends.** Without a map, a straight plan toward a point behind a wall drives into
   whatever pocket lies in between, and the 10 s obstacle memory forgets the pocket's
   walls.

## Design: teach and repeat

On the way out the robot drove a route that is known to be passable, because it just
drove it. The way back repeats that route in reverse.

### Breadcrumbs (brain, in `hmc/`)

While following, the brain drops a breadcrumb at its own believed position every 0.5 m.
When a new breadcrumb lands within 0.5 m of an older one that is not among the last
three, the loop in between is cut out (the robot circled while searching or waiting).

The way back drives to the breadcrumbs in reverse order with the same planner and
obstacle handling as following: A* around what the sensors see now, so a chair that
moved onto the route is driven around. A breadcrumb counts as reached within 0.4 m. If
the next breadcrumb cannot be reached for 5 s, the robot aims at the one after it (up to
three ahead). If none of them can be reached for 20 s, it stops and says "Please let me
pass." and keeps trying.

The last breadcrumb is the start itself. The robot stops there within 0.3 m of its
believed position, turns to the heading it started with, and says it has arrived.

### Position correction (robot only, in `base/`)

Breadcrumbs fix the dead-end problem, not the drift. For drift the robot keeps what it
saw on the way out: with every breadcrumb it stores the LiDAR scan taken there
(a keyframe). On the way back, whenever it is within 1.0 m of a keyframe and facing
within 30° of the reverse or the original heading, it matches the current scan against
the stored one (the scan matcher built for the speed ladder in the follow plan) and
corrects its believed pose by the difference.

A match is used only when it is trustworthy: at least 60 % of the points agree within
5 cm, and the correction is smaller than 0.5 m and 20°. People standing in the scan
lower the share of agreeing points, so a crowded keyframe is skipped, not forced. With
no usable match the robot continues on odometry alone and says so in its log.

The simulation has no scans. Its stand-in is the existing `--odometry slam` model, which
already represents "a correction that leaves about 5 cm". So the brain is the same and
only the source of the pose differs, as with following.

This is a map of the route made during the run, which the rules allow. It is not a map
of the area made beforehand.

### The queue (brain, in `hmc/`)

The simulation already has this logic in `task.py`; it moves behind the same kind of
interface as the follower.

- **On the way out**, remember the crowd: people who stood still for at least 6 s in a
  group (within 1.5 m of each other) within 4 m of the robot. They are remembered by
  look, not by track number.
- **On the way back**, the queue is: people who are walking, who look like someone of
  the remembered crowd, and who are closer to the start than the robot. The robot
  follows the nearest of them, 1.0 m behind, in the direction that person walks.
- It leaves the queue 2 m from the start, or when the person it follows stops walking,
  or when no queue member has been seen for 3 s, and then goes on along the breadcrumbs.

For this the robot's `Senses` must also carry the look of every person the camera saw,
not only the operator's. The Jetson already returns a look for every detected person.

Without the Jetson link there are no looks, so no queue: the robot just drives back.
That keeps the 200-point bonus and gives up the 300-point one.

### What starts and ends the way back

- Start: `Follower` stays in control until the owner presses Enter on the notebook
  (later: the arm reports "bag held", or a voice command). The robot says
  "I will go back to the start now."
- End: the robot is at the last breadcrumb. It says "I am back at the start." and stops.
  Putting the bag down is the arm's work, later.

## Ready-for-competition criteria

Each scenario five times. "Error at the start" is measured by the owner with a tape
from a mark on the floor under the robot's turning centre at the start.

1. Inside, a route of about 10 m with two turns: error at the start at most 0.5 m, every
   run, nothing touched.
2. Corridor outside the room, about 30 m with the door and a corner: the same.
3. The same corridor route with the position correction switched off: record the error,
   to know what the correction is worth. No pass limit.
4. A chair put on the route after the robot passed: driven around, start reached.
5. The route blocked completely for 15 s, then cleared: the robot waits, asks to pass,
   then finishes.
6. Queue: three teammates stand as a group near the route on the way out for at least
   6 s, then walk back in a line. The robot joins within 2 m behind the last one, never
   falls more than 3 m behind, touches nobody, and reaches the start.
7. A passer-by walks toward the start ahead of the robot while no queue walks: the robot
   does not follow them.
8. The Jetson link cut before the way back: the robot returns without the queue.

## Calls made by the planner, for the owner to check

- Teach and repeat with scan keyframes instead of running a full SLAM package. It needs
  no ROS, reuses the scan matcher the follow plan already builds, and only has to know
  the one route the robot drove. A full SLAM system would also work but is a much larger
  piece to run next to the direct serial drivers.
- Breadcrumb spacing 0.5 m, reached within 0.4 m, stop at the start within 0.3 m.
- The simulation's way back changes from "straight to the start" to "along the
  breadcrumbs", so the simulation exercises the code the robot runs. Its evidence
  folder must be regenerated, and every map must still end with 1115 points.
- The queue walks at its own pace. If the robot's top speed (from the speed ladder) is
  below the queue's pace, the robot will fall more than 3 m behind and lose the bonus;
  the rules let the robot ask the operator to slow down, not the queue. This is measured
  in criterion 6, not solved here.

## Left to the builder

- The scan-match thresholds, tuned on recorded runs.
- How keyframes are stored and searched.
- The exact wording of the spoken lines.
