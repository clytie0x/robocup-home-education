# Virtual operator

The real robot drives in the real lab and follows a person who exists only on the owner's PC. The robot can then test the Help Me Carry follow and return by itself, episode after episode, with nobody walking in front of it.

| What | Where |
|---|---|
| Robot side | `base/pi/virtual_operator.py`, hooked into `run_follow.py --virtual-operator PORT [--virtual-allow PC]` |
| Console option | Task tab, "คนที่ตาม: คนเสมือน" (`POST /api/task/start {"virtual": true}`), PIN needed |
| PC app | `base/pc/virtual_operator/app.py` (logic in `sim.py`, ssh control in `remote.py`, page `index.html`) |
| Tests | `base/pi/tests/test_virtual_operator.py`, `test_virtual_run.py`, `test_console_virtual.py`, `test_bag.py`; `base/pc/virtual_operator/tests/test_sim.py`, `test_app.py` |

## What it tests, and what it does not

Tested for real: the wheels, the odometry, the real LiDAR (walls, furniture, real people), the leg tracker and the senses rules, the brain (follow, the gap, the search, RETURN along the breadcrumbs with the keyframe corrections, the queue), the safety stop, the cable rules, the console and the whole timing of the 10 Hz cycle.

Not tested: **the camera detector is bypassed.** The Jetson's people boxes and looks are replaced by a made-up answer (one person, conf 0.9, the same look every cycle). Lighting, a person half out of the image, a wrong look, a missed detection and the Jetson link itself are not exercised. Voice is not either: the PC says the words, the robot gets them as the page's task buttons. The virtual person's legs are perfect circles that swing evenly; real legs, skirts and bags look different to the LiDAR. A real person walking into the room is still a real obstacle and a real candidate for the senses, but the camera never sees them, so the robot can never enrol a real person in this mode.

## How the person gets into the robot

The PC sends `{"t", "x", "y", "th", "speaking", "pointing"}` as one UDP datagram 10 times a second to the Pi's port 5610, in the run's odometry frame (the frame of the record's `pose`). Each cycle, before the senses read, `virtual_operator.apply`:

1. LiDAR: two leg circles (radius 0.06 m, centres 0.18 m apart across the heading, swinging up to 0.12 m with the distance walked) are cut into the real scan at the real scan's own bearings. On a bearing where a real return is nearer, the virtual return is dropped (the real thing hides the leg). Where the leg is nearer, it replaces the real return (the leg hides the wall), as a real person would.
2. Camera: one `Seen` at the person's bearing from the camera when the person is inside the 58.6 degree view, within 5 m, and at least half of the leg returns survived step 1; else an empty answer. Real camera people are always dropped.
3. Words: a new word is one command, as the voice flow sends it: follow is follow, stop is wait, done and back are return, bag is bag (what the flow's `find_bag` sends: look for the pointed bag and drive up to it, run_follow's BAG phase, `base/pi/bag.py`; it releases the page Wait that stop put on, since the same page asks the robot to move). Each fresh datagram is also the page heartbeat, so the console's silence hold does not stop a run led by the PC.

4. Pointing: `"pointing": [x, y]` (a floor spot in the same frame, or null) goes to the record as `"virtual_pointing"`; the BAG phase uses it in place of the camera's pointing (`base/jetson/pointing.py`), from the person's shoulder.
5. Bags: `"bags": [[x, y], ...]` (at most 4, same frame, optional) are 0.065 m circles cut into the scan the senses read like the legs (a real return nearer on the same bearing wins). The BAG phase finds, chooses and approaches them as real bags, so the whole find and approach runs on the real robot with no real bag. Like the legs, the last-resort stop never sees them, and while creeping to a virtual bag no real point is left out of the stop (only a real locked bag may come closer than the clearance).

No datagram for 0.5 s: the person is gone (and so is the pointing). The senses get the real scan and no camera answer, and the robot behaves as with a lost operator (it holds, then searches).

The record of every cycle has `"virtual"`: the person's pose, whether the camera saw them, how many leg returns were shown and hidden, the word, the sender and the real Jetson link.

## Safety

- `safety.must_stop` decides on the **real** points only (plus the depth camera), with `stop_clearance` 0.10 as always. The virtual legs never reach it. Choice: the virtual person is a test aid for the brain. The brain keeps its follow gap to the person as to a real one, but the last-resort stop is neither weakened (no real return is removed for it) nor triggered by someone who is not there. A virtual person walking backward into the robot is not a stop; nobody can be hurt by it.
- The record's `nearest`, the scan matching, the keyframes and the saved scans are the real ones too.
- Only datagrams from 127.0.0.1, the PC address the console was asked from (or given) and Tailscale (100.64.0.0/10) are taken; anything malformed is counted and dropped.
- The console option needs the console PIN (also for a dry run), because whoever sends the person steers the robot. Over ssh the login key is the gate; the PIN never leaves the Pi.
- A Drive run is still started under `tools/safe_run.py`, at 0.08 or 0.12 m/s, never above `params.json` `max_speed`.
- The auto loop presses the console's STOP when the console reports a wheel fault or a lost link for 2 s, when the run ends by itself, when the robot's body comes within 0.05 m of anything (from the run's scan, measured from the 0.40 m square body), or when the record's `cable_deg` passes 150 degrees. It is not an emergency stop: the motor cut-off on the robot is. Keep the power cable and the room in view while it runs.

## Running it

Pi: nothing new to start. In the console's Task tab pick "คนเสมือน" (optionally the PC's address; empty takes the computer that pressed Start), or let the PC app start the run.

PC (Windows, Python 3.13, no admin, no packages, the OpenSSH client with the key for xohm@ already set up):

```
python base/pc/virtual_operator/app.py                      # runs over ssh (default)
python base/pc/virtual_operator/app.py --control console    # runs through the console and its PIN
```

then open http://127.0.0.1:8095/. The page loads three.js 0.160.0 from unpkg.com, so the PC needs the internet; the robot does not.

Control over ssh (default, no person needed): the app logs in as `xohm@192.168.31.21` (Tailscale `100.73.128.36` when the Wi-Fi does not answer) with `ssh -o BatchMode=yes host bash -s` and a script on stdin (`remote.py`). Start: when the console answers on the Pi, through its API with the PIN read from `~/.config/hmc-console-pin` on the Pi (the console holds the camera and LiDAR, so a run beside it could not open them); otherwise `run_follow.py ... --remote --virtual-operator 5610 --virtual-allow <this PC>` directly, detached with `setsid` and stdin from /dev/null, a Drive under `tools/safe_run.py`, output in `~/hmc-base/logs/virtual/`, the process group id in `logs/virtual/run.pid`. Stop: the console's STOP, run_follow's own end (`{"end": true}` to `127.0.0.1:8080/control`, which sends S#), then SIGTERM and after 3 s SIGKILL to the group in the pid file; nothing is killed by a name pattern. Status and the LiDAR come through `ssh -N -L 127.0.0.1:18080:127.0.0.1:8080` from run_follow's own page; when a run's page stops answering, its end text is read from the last line of the newest `logs/<start>/log.jsonl`.

Control through the console (the phone's way): pick "Wi-Fi แล็บ" (http://192.168.31.21:8081) or "Tailscale" (http://100.73.128.36:8081), or type one (`--console URL`), and type the console PIN once. It stays in the app's memory only; restarting the app forgets it.

Then pick Dry run or Drive, the speed and the length, then "เริ่มงานคนเสมือน", or go straight to "เริ่มรอบอัตโนมัติ" (it starts runs itself).

Click the floor to send the person there (A* on the map, 0.3 m from every wall, away from the robot). The buttons follow, stop, done and back make the person say the word. "หยุด" stops the person and the auto loop and stops the run (over ssh or with the console's STOP).

"วางถุงที่คลิก" puts a virtual bag at the next clicked floor spot (at most 3), "ล้างถุง" removes them. "ชี้ถุง" points at the bag the robot reports (the record's `"bag": [x, y]`, odometry frame: the bag the BAG phase chose), else the next floor click is the spot; "ชี้จุดที่คลิก" points at the next clicked spot (click on a virtual bag to point at it). The person points for 5 s and faces the spot. The robot panel shows the BAG task (`"bag_task"`: the chosen bag's sentence, then reached with x, y or failed with its code) and a green ring on the chosen bag.

A bag test without the Jetson: place one or two bags, walk the person beside them, say stop (the robot holds), point at a bag ("ชี้จุดที่คลิก", then click it), then say bag within the 5 s of pointing. The robot turns, drives and creeps until the bag centre is 0.31 m ahead of its turning centre. Then say back for the way back.

Script API (127.0.0.1:8095 only; every POST needs `Content-Type: application/json` and `X-VO-App: 1`):

| Call | What |
|---|---|
| `GET /api/auto/status` | the loop (`auto.on`, `step`, `message`), the robot's record fields, the person, the last episodes |
| `POST /api/auto/start {"drive", "speed", "minutes"}` | the auto loop (the settings are optional; speed 0.08 or 0.12, 1 to 30 min) |
| `POST /api/auto/stop` | the loop off, the person stops, the run gets STOP |
| `POST /api/run/start`, `POST /api/stop` | one run with the current settings; STOP |
| `POST /api/goal {"x", "y"}` | walk there (409 when the map has no way) |
| `POST /api/say {"word"}` | follow, stop, done, back or bag |
| `POST /api/point {"x", "y"[, "seconds"]}`, `{"bag": true}`, `{"clear": true}` | point at a floor spot |
| `POST /api/bags {"x", "y"}`, `{"clear": true}` | a virtual bag there (409 past 3), or none |
| `POST /api/config {"control": "ssh"/"console", "console", "pin", "speed", "run"}` | settings |

```
curl -X POST -H "Content-Type: application/json" -H "X-VO-App: 1" -d "{\"drive\": true, \"speed\": 0.08, \"minutes\": 10}" http://127.0.0.1:8095/api/auto/start
```

The room map comes from the run's own LiDAR (`/scan` with the pose of the same cycle), 5 cm cells, built while the robot looks around. The person only walks on floor the LiDAR has seen. A bearing with no return within 4 m counts as open floor up to 4 m (the console sends no farther points), so a dark or glass surface can look open: watch the first walks.

Auto loop: stand 1.0 m in front of the robot until it locks, say follow, walk to a random reachable spot 2 to 4 m away at the walking speed, say stop once the robot is within 1.2 m, wait 5 s, say back, wait until the run ends with "done", then start the next run. Each episode is a row in `base/pc/virtual_operator/episodes.csv` (start time, start and goal, walk length, time until the robot was within 1.2 m, the nearest gap between the body and anything, seconds the robot stood still while it should have moved, result). The file is local test data.

## Walking speed

Default 0.24 m/s, the median of the owner's real walking while the robot followed (the run logged as `run-023855/log.jsonl`, operator positions over 2 s windows, the 433 of 2872 windows faster than 0.15 m/s; 3 s windows give 0.23, 1 s windows 0.27). The owner walked slowly because the robot drives at most 0.12 m/s. The page can set another speed.
