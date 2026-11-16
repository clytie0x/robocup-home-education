"""The gate is base/jetson/console_gate.py, the one gate of both hosts: this loads it into this module from jetson/
next to the Pi code (deploy.sh copies it there) or from ../jetson (the laptop), as ports.py does. The Pi logs its
maintenance lines in the console's own log."""
from pathlib import Path

_here = Path(__file__).resolve().parent
for _path in (_here / "jetson" / "console_gate.py", _here.parent / "jetson" / "console_gate.py"):
    if _path.exists():
        exec(compile(_path.read_text(encoding="utf-8"), str(_path), "exec"))
        break
else:
    raise ImportError("console_gate.py not found in jetson/ or ../jetson/ (deploy.sh copies it)")
LOG = _here / "logs" / "console" / "commands.jsonl"  # robotctl log and the console page read this one
