"""Every TCP/UDP port of base/pi and base/jetson, one file for both hosts (base/docs/communication.md).
The Jetson imports it from here; the Pi gets it as ~/hmc-base/jetson/ports.py (deploy.sh) through base/pi/ports.py.
A new port goes here, in HOSTS, and in the table of communication.md, on a number its host does not use yet."""

# Pi
CONSOLE = 8081  # Pi, console.py, HTTP: the page, robotctl
DASHBOARD = 8080  # Pi 127.0.0.1, run_follow.py's dashboard.py, HTTP: the console's task buttons, STOP, status
PLAN = 5603  # Pi 127.0.0.1, plan_server.py, TCP: run_follow's paths
COMMAND = 5604  # Pi, run_follow.py --remote (command_server.py), TCP JSON lines: the Jetson base_bridge's commands
VIRTUAL = 5610  # Pi, run_follow.py --virtual-operator, UDP: the PC's simulated person (virtual_operator.py)

# Jetson
DETECTOR = 5601  # Jetson, server.py, TCP length-prefixed JSON + JPEG: frames in, people back
SLAM = 5602  # Jetson, slam_bridge.py, TCP length-prefixed JSON: scans in, map correction back
HOLD = 5605  # Jetson 127.0.0.1, base_bridge.py's HoldListener, UDP: agent.py's b"hold"
LOOK = 5606  # Jetson, front_camera.serve (started by bag_arm.py), TCP: bag top for the Pi's bag.py
ARM_CMD = 5607  # Jetson 127.0.0.1, bag_arm.py's command server, TCP JSON line: agent.py's arm commands
WHISPER = 8080  # Jetson 127.0.0.1, whisper-server (agent program "whisper"), HTTP: speech to text
WEBCAM = 8082  # Jetson, the arm team's ~/so100_ws/webcam_view.py, HTTP JPEG: arm camera frames
AGENT = 8090  # Jetson, agent.py, HTTP JSON: programs, /arm, status for the console

HOSTS = {  # the names of the ports each host listens on; a number twice on one host is a test failure
    "pi": ("CONSOLE", "DASHBOARD", "PLAN", "COMMAND", "VIRTUAL"),
    "jetson": ("DETECTOR", "SLAM", "HOLD", "LOOK", "ARM_CMD", "WHISPER", "WEBCAM", "AGENT"),
}
