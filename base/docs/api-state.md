# GET /api/state — mission

For the console agent. The page already polls `GET /api/state`. The Help Me Carry executive (`hmc.flow.HmcFlow`) adds one object, `mission`. `POST /api/task/start` resets and starts it, and drives `run_follow` as a step. `POST /api/stop` stops the base and the arm, records `OPERATOR_STOP`, and does not stop-all the Jetson programs.

`POST /api/mission/hear` with `{"text": "..."}` and `POST /api/mission/event` with `{"event": "name|CODE|detail"}` (or a dict with `name`, `code`, `detail`) feed the executive. Both need the same page header and PIN as the other commands.

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
| `limit_s` | The cap: the asked length, and never more than 300 |
| `reason` | `null`, or `{code, en, th, fix, detail, level}` |
| `carrying` | The arm is holding the bag |
| `car_pose` | `{x, y, heading}` after the operator confirms the car, else `null` |
| `say` | Last line for the operator |
| `side` | `left` or `right` when the operator named a side |
| `log` | Path of `events.jsonl`, or `null` |

`events.jsonl` is one JSON object per line: `ts` (ICT, offset `+07:00`), `run_id`, `src` (`hmc.flow`), `state`, `event`, `code`, `detail`, `visible`, `say`, `level`, `outcome`.

`reason.code` is sticky. Each code has English (`en`), Thai (`th`), and a suggested fix (`fix`). The set includes `ODOM_STALE`, `BASE_LINK_DOWN`, `NO_LINK`, `TIME_LIMIT`, `OPERATOR_LOST`, `PICK_FAILED`, `OPERATOR_STOP`, `WHEEL_FAULT_02`, `WHEEL_FAULT_03`, `REID_DOWN`.
