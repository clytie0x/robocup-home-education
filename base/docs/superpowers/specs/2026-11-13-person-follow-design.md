# Person following for Help Me Carry: design

Status: approved in conversation by the owner on 2026-11-13, then extended by the owner's
request for one method shared by the simulation and the robot.
Hardware facts: `base/docs/hardware-assessment-2026-11-13.md`.

## Why

In Help Me Carry (rulebook 2025.1, section 5.1) the robot follows the operator out of the
arena, past people who walk by and stand in groups. On 2026-11-13 the robot followed with
a camera-only detector on the Pi and lost the operator in 67 % of the control cycles. A
step out of the 58° camera view was enough to lose them for good.

There are also two different followers today. The simulation in `hmc/` plans with A* from
what it sees and recognises the operator by look. The robot ran `pi-drive`, a reactive
rule written separately. A result in one says nothing about the other. This design
removes that split.

## Goals

1. Follow without a break, also when the operator is outside the camera view.
2. Follow the right person when others pass or cut in.
3. Touch nothing and nobody.
4. One decision code for the simulation and the robot, so simulation results mean something.

## Scope

In: following only, inside the arena and outside it.

Out, for later work: bags and the arm, voice commands, the way back, mapping.

Failures even if everything else works:
- the robot touches a person or an object;
- the robot drives faster than it can brake;
- the robot silently follows someone else. When unsure it stops and waits.

## Decisions the owner made

- LiDAR measures distance. The camera says who is who and where they are.
- YOLO and re-identification run on the Jetson.
- The robot may stop close to the operator: about 0.6 m from its front edge.
- The depth image is also used to stop for things the LiDAR cannot see.
- When something blocks the way, the robot plans a way around it.
- The robot may drive faster than 0.3 m/s, as fast as measured braking allows.
- TensorRT may be installed on the Jetson with `apt`.
- The Jetson is shared with other teams, is not on the robot today, and may be on a LAN
  cable at the competition.

## Design

### One brain, two bodies

The decision code lives in `hmc/` and stays pure Python (standard library only, no
hardware). It is the code the simulation already uses: operator tracking and the look
check from `task.py`, the A* planner and `Navigator` from `navigation.py`, the look
memory from `appearance.py`, and the motion limits of `world.Robot`.

That code is moved behind one small interface, a `Follower` that is given a body and one
`Senses` record per cycle:

- `Senses`: the time, the operator's position if the tracker has them, the operator's
  look if the camera saw them this cycle, the other people, and the obstacles.
- body: pose, speed, and the `drive_toward`, `turn_toward`, `brake` methods of
  `world.Robot`.

In the simulation `Senses` is filled from the simulated world, as today. On the robot it
is filled from the LiDAR, the depth image and the Jetson. The robot's body is a
`world.Robot` whose pose is overwritten with measured odometry each cycle; after the
brain's step, its `speed` and `turn_speed` are the command sent to the wheels. So the
control law is not written twice.

Hardware code lives in a new top-level folder `base/`, because `hmc/AGENTS.md` forbids
hardware code inside `hmc/`.

### Pi: safety and driving. It must work alone when the Jetson is gone

- LiDAR leg tracker: clusters the scan into person-sized objects and keeps one track on
  the operator through 360°. This is what says where the operator is.
- Depth obstacles: depth points between 5 cm above the floor and the robot's height,
  inside 3 m, become obstacles. They are remembered for 10 s after leaving the view
  (the same `MEMORY_KEEP` the simulation uses), because the camera looks forward only.
- Obstacles for the planner: LiDAR points and depth points as small circles, other
  people as person-sized circles.
- Last-resort stop, separate from the planner: if anything is inside the braking
  distance in the direction of travel, send `S#`, whatever the planner asked for.
  Braking distance = v² / (2a) + v × delay + 0.15 m, with measured a and delay.
- Camera frames go to the Jetson as JPEG with their capture time.
- If the Jetson has not answered for 0.5 s, the robot keeps following on the unbroken
  LiDAR track. If that track then breaks, it stops and waits. It never picks a new
  person without the Jetson.
- No depth image (sunlight outside the arena) means the top speed drops to the value
  the LiDAR alone can protect. It never means "nothing there".

### Jetson: who is who

- A person detector (YOLO family, TensorRT FP16).
- A re-identification network gives each detected person a look vector.
- It returns every person in the frame: bearing, box, confidence, look vector, and the
  capture time of the frame.

### Knowing the operator

- Start: the owner presses a key on the notebook. The person in the middle of the
  image, closer than 2.5 m, is learned for 20 frames (`OPERATOR_LEARN`). If more than
  one person is that close, the robot refuses and asks to try again.
- While the LiDAR track is unbroken the robot keeps following; the camera look is
  checked whenever the camera sees the operator, and a look that does not match drops
  the track (`OPERATOR_MATCH`).
- After a break, only a camera detection whose look matches the operator can start a
  new track. Until then the brain's search runs: drive to where the operator was last
  seen, turn on the spot, then ask them to come back (a printed line until the voice
  work is connected) and repeat the turn every 10 s.

### Speed

- A speed ladder comes first: 0.3, 0.4, 0.5, 0.6 m/s, three runs each, measured with
  LiDAR scan matching. Each step records real speed, braking distance after `S#`, and
  how straight the line was.
- The top speed is the highest step that brakes within 0.4 m and drifts less than 5 cm
  per metre.
- The planner already slows down for what is ahead (`free_ahead`), so the robot is fast
  only where the way is clear.

### Link

Plain TCP between the Pi and the Jetson, no ROS. Only the newest frame is sent; a frame
that waited is dropped. Old camera answers are corrected with the robot's own motion
since the capture time.

## Ready-for-competition criteria

Every scenario is run five times, inside the arena and again outside it.

1. Five minutes of walking around: the operator track is held at least 95 % of the time.
2. The operator walks to the side of the robot and behind it: followed every time.
3. A second person crosses between robot and operator: no switch, in any run.
4. A second person stands next to the operator, then they walk apart: the robot follows
   the operator or stops. It never follows the other person.
5. The operator hides for 5 s and returns: followed again within 3 s of being seen.
6. Through a door and around a table corner: followed without touching anything.
7. A table with an overhanging top in the way: the robot stops or goes around. It never
   drives under it.
8. In all runs the closest approach to any person or object, from the LiDAR log, is at
   least 0.3 m.
9. The Jetson link is cut while driving: nothing is touched and nobody else is followed.

## Calls made by the planner, for the owner to check

- "Track held" in criterion 1 means the LiDAR track on the operator is unbroken. The
  operator does not have to be inside the camera image.
- Code location: `base/` in this repository for the Pi and Jetson code; on the robot it
  is deployed to `~/hmc-base` (Pi) and `~/hmc-follow` (Jetson), each with its own
  virtual environment. `~/pi-drive` and the other teams' files are left as they are.
- First frame size 320 x 240 (what the camera gives today). 640 x 480 is tried if
  re-identification is weak at 4 m.
- The robot is planned as a circle of radius 0.29 m (the 0.40 m square plus 1 cm).

## Left to the builder

- The exact detector and re-identification models and input size, chosen by measuring
  at least 15 frames per second on the Jetson while the other teams' software runs.
- The look thresholds, set from recorded data.
- The message layout on the TCP link.
