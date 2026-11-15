# Systemd user units for the robot

The same two boot jobs as the user crontab, run by each machine's own systemd, without sudo
and without a login. The console and the agent keep their log files under
`~/hmc-base/logs/console/console.out` and `~/hmc-follow/logs/agent.out`.

| File | Machine | What it runs |
|---|---|---|
| `hmc-console.service` | Pi | `~/hmc-base/.venv/bin/python -u console.py` in `~/hmc-base`, port 8081 |
| `hmc-agent.service` | Jetson | `python3 agent.py` in `~/hmc-follow`, port 8090 |
| `hmc-start-all.service` | Jetson | one `POST /start-all` after `GET :8090/status` answers; off by default |
| `install.sh` | either | copies the units, enables them, turns on lingering, prints the redundant crontab lines |

The console unit does what `tools/run_console.sh` does, without its while loop: `systemd`
restarts the console after 2 s. The unit and `tools/run_console.sh` use the same lock file
(`/tmp/hmc-console.lock`), so a copy started by the crontab script and the service can never
run together.

`deploy.sh --restart` works with either: on the Pi it ends the console (`.venv/bin/python -X
faulthandler -u console.py`, the same command line in both) and the keeper starts it again; on
the Jetson it runs `systemctl --user restart hmc-agent.service` when that unit is enabled, else
it starts a nohup copy as before. Both units restart without a start limit
(`StartLimitIntervalSec=0`).

## Install

`deploy.sh` does not carry this folder (it deploys `base/pi` and `base/jetson` only). Copy
`base/systemd/` to the machine once, then run the installer as the robot's user:

```sh
sh install.sh pi                     # on the Pi, user xohm
sh install.sh jetson                 # on the Jetson, user robotpsu
sh install.sh jetson --start-all     # optional, see below
```

The installer copies the units to `~/.config/systemd/user/`, runs `systemctl --user
daemon-reload`, enables them and runs `loginctl enable-linger`. It does not start them, does
not edit the crontab and removes nothing. It prints the crontab lines that are now redundant.

Then, on that machine:

```sh
systemctl --user start hmc-console.service     # Pi
systemctl --user start hmc-agent.service       # Jetson
systemctl --user status hmc-console.service    # or hmc-agent.service
```

Stop the crontab copy first, or the new service and the old copy compete for the same port
and lock. The installer deliberately does not kill anything. On the Pi stop the wrapper before
the console (`pkill -f '[r]un_console.sh'`, then `pkill -f 'console.py'`); on the Jetson
`pkill -f 'agent.py'` stops the programs the agent started before it exits.

## The optional Start all unit

`hmc-start-all.service` is a oneshot that waits up to 120 s for the agent's `GET /status`,
then sends `POST /start-all`. It is disabled by default because Start all starts seven
programs, the arm stack among them, and a person should see the first one work. Enable it only
after the paths at the top of `agent.py` (`VOICE_ROOT`, `VOICE_WS`) are checked on the robot and
Start all works from the console:

```sh
systemctl --user enable --now hmc-start-all.service
```

It `Requires=hmc-agent.service`, so starting it also starts the agent. To turn it off again:
`systemctl --user disable --now hmc-start-all.service`.

## Things to know

- These are user units. `sudo systemctl` does not see them; always use `systemctl --user`.
  `%h` in a unit expands to the user's home, so nothing here hard-codes `/home/xohm` or
  `/home/robotpsu`.
- Without `loginctl enable-linger` the user manager ends at logout and nothing starts at boot.
  The installer turns it on; if it cannot, it prints the `sudo loginctl enable-linger` command.
- Nothing starts a task or the Jetson programs by itself. The console waits for Start task,
  the agent for Start all (unless the optional unit is enabled).
- After the agent is killed outright (`kill -9`, OOM) and `Restart=always` starts it again, the
  programs the old agent started keep running. The new agent lists them as "outside" and Start
  all refuses to start another copy (`agent.py`, `_outside_error`). Stop those copies at a
  Jetson terminal, then Start all. A clean `systemctl --user restart hmc-agent` sends SIGTERM,
  and the agent stops its own programs before it exits, so prefer that over killing it.
- A console left running by the old crontab makes the service exit at once and restart every
  2 s, because of the shared lock. That is the lock doing its job, not a crash to debug.
- The log files do not rotate, exactly as with the crontab lines. Move an old file aside
  before a long day if it gets large; `journalctl --user -u hmc-console` also has the service's
  own messages.
- These units were checked here as unit files only. They have not been installed or started on
  a robot in this change. Run the install and one `systemctl --user status` on each machine
  before relying on them at the venue.
