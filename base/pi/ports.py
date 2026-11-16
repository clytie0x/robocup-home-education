"""The port table is base/jetson/ports.py, the one source for both hosts: this loads it into this module from
jetson/ next to the Pi code (deploy.sh copies it there) or from ../jetson (the laptop), as bag.py loads pointing.py."""
from pathlib import Path

_here = Path(__file__).resolve().parent
for _path in (_here / "jetson" / "ports.py", _here.parent / "jetson" / "ports.py"):
    if _path.exists():
        exec(compile(_path.read_text(encoding="utf-8"), str(_path), "exec"))
        break
else:
    raise ImportError("ports.py not found in jetson/ or ../jetson/ (deploy.sh copies it)")
