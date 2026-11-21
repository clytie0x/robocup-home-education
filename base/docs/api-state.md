# GET /api/state — mission

For the console agent. The page already polls `GET /api/state`. The Help Me Carry executive (`hmc.flow.HmcFlow`) adds one object, `mission`. `POST /api/task/start` resets and starts it, and drives `run_follow` as a step. `POST /api/stop` stops the base and the arm, records `OPERATOR_STOP`, and does not stop-all the Jetson programs.

`POST /api/mission/hear` with `{"text": "..."}` and `POST /api/mission/event` with `{"event": "name|CODE|detail"}` (or a dict with `name`, `code`, `detail`) feed the executive. Both need the same page header and PIN as the other commands.

During a full mission run (`mission` is `full`) the console relays the Jetson flow program through the agent's `POST /flow`; in a `follow_only` or `bag_only` run it stops the flow program as before, so speech does not steer those. At the start it parks it (`own`, a lease of `FLOW_LEASE` = 5 s), then every 0.25 s reads what it heard (`heard`, which renews the lease) and feeds each line to the executive, sends the executive's `say` lines in order, and releases it once when the run ends (a terminal state, STOP, or the run job's end, except while the executive is on the way back or dropping the bag: run_follow ends in the cycle it reaches the start, and the drop and "Here is your bag." come after). An agent without `/flow`, or a flow program that does not answer, is stopped as before, with no speech, and the run's `events.jsonl` gets an `ignored` line `voice_relay_off`. A failed read never ends the run: `voice_relay_lost` and `voice_relay_ok` are logged once each. The executive's `pick_bag` and `drop_bag` run as the console's bag job in a thread of their own (the 5:00 clock keeps ticking) and come back as `grab_success`, `grab_failed|CODE|message`, `drop_done` or `drop_failed|CODE|message` (`TIMEOUT` when the agent did not answer). The drop goes to `so100_ik.reachable_drop` (the page's drop default; the executive's 0.31, 0.0 only when that module cannot load), and a drop is taken after run_follow has ended, since nothing can move the base then. run_follow's page ends with the run, often before the console's next read: the console then reads the run's last `end` record (with `at_start`) from the newest run log, so the executive still sees the arrival and the reason. The run's bag task (`record.bag_task` of run_follow's status) comes back once per task that a `find_bag` asked for: `bag_chosen|SIDE|say`, then `bag_reached|CODE|x,y[,top=T][,yaw=Y]` or `bag_failed|CODE|message`, as `base/jetson/base_bridge.py` sends them.

While `mission.active` is true, or a task job is running, `POST /api/arm` (home, rest, center, torque, pick_bag, drop_bag, bagpick) answers 409. The bag-only worker still calls the arm itself.

## mission

| Field | Meaning |
|---|---|
| `active` | True until the run reaches a terminal state |
| `run_id` | Log folder name. `events.jsonl` is in that folder under the log root |
| `state` | `IDLE`, `PRECHECK`, `READY`, `ENROLLING`, `FOLLOWING`, `LOST_SEARCH`, `CONFIRM_CAR`, `AT_CAR`, `POINTING`, `PICKING`, `HANDOVER`, `RETURNING`, `DROPPING`, `DONE`, `FAILED`, `ABORTED` |
| `outcome` | `null`, `SUCCESS`, `FAILED`, or `ABORTED` |
| `step_index` | 1..`step_total` for the current step, 0 when `IDLE`. `LOST_SEARCH` shares the follow step and `HANDOVER` shares the pick step |
| `step_total` | 11 |
| `elapsed_s` | Seconds since start |
| `remaining_s` | Seconds left on this run's clock |
| `limit_s` | The cap: the asked length, and never more than 300. In a `full` mission run_follow gets `RETURN_S` (180 s, the executive's `RETURN_BUDGET`) more, so it still drives the way back after the clock |
| `reason` | `null`, or `{code, en, th, fix, detail, level}` |
| `carrying` | The arm is holding the bag |
| `car_pose` | `{x, y, heading}` after the operator confirms the car, else `null` |
| `say` | Last line for the operator |
| `side` | `left` or `right` when the operator named a side |
| `log` | Path of `events.jsonl`, or `null` |

`events.jsonl` is one JSON object per line: `ts` (ICT, offset `+07:00`), `run_id`, `src` (`hmc.flow`), `state`, `event`, `code`, `detail`, `visible`, `say`, `level`, `outcome`.

`reason.code` is sticky. Each code has English (`en`), Thai (`th`), and a suggested fix (`fix`). The set includes `ODOM_STALE`, `BASE_LINK_DOWN`, `NO_LINK`, `TIME_LIMIT`, `OPERATOR_LOST`, `PICK_FAILED`, `OPERATOR_STOP`, `WHEEL_FAULT_02`, `WHEEL_FAULT_03`, `REID_DOWN`.
