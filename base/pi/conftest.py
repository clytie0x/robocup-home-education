"""pytest from base/pi on the laptop or in CI: the hmc package is ../../hmc/src here (on the Pi deploy.sh puts it
in ~/hmc-base/hmc, next to this file, so nothing is added there)."""
import sys
from pathlib import Path

_here = Path(__file__).resolve().parent
_hmc = _here.parent.parent / "hmc" / "src"
if not (_here / "hmc").exists() and _hmc.exists():
    sys.path.insert(0, str(_hmc))
sys.path.insert(0, str(_here))
