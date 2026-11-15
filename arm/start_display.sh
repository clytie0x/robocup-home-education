#!/bin/bash
# Virtual screen :1 for RViz, viewable in a browser through noVNC (localhost only; use an SSH tunnel).
# Usage: ~/so100_ws/start_display.sh      then on Windows:  ssh -N -L 6080:localhost:6080 robotpsu@<jetson>
#        and open http://localhost:6080/vnc.html
set -e
if ! pgrep -f "[X]vfb :1" >/dev/null; then
  setsid Xvfb :1 -screen 0 1600x950x24 >/tmp/xvfb.log 2>&1 &
  sleep 1
fi
export DISPLAY=:1
pgrep -x openbox >/dev/null || (setsid openbox >/tmp/openbox.log 2>&1 &)
ss -ltn | grep -q "127.0.0.1:5900 " || (setsid x11vnc -display :1 -localhost -nopw -forever -shared -quiet >/tmp/x11vnc.log 2>&1 &)
ss -ltn | grep -q "127.0.0.1:6080 " || (setsid websockify --web /usr/share/novnc 127.0.0.1:6080 127.0.0.1:5900 >/tmp/novnc.log 2>&1 &)
sleep 1
echo "display :1 ready -> http://localhost:6080/vnc.html (through the SSH tunnel)"
