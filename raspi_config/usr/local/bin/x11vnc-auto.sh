#!/bin/bash
# warten bis Xorg läuft
for i in $(seq 1 60); do
  PID=$(pgrep -o -x Xorg) && break
  sleep 2
done
[ -z "$PID" ] && { echo "Kein Xorg gefunden"; exit 1; }

# Auth-Datei aus Xorg-Cmdline
AUTH=$(tr '\0' '\n' < /proc/$PID/cmdline | grep -A1 '^-auth$' | tail -n1)

# Display aus Socket, den dieser Xorg hält
DISP=$(ss -xlp | grep "pid=$PID," | grep -oP '/tmp/\.X11-unix/X\K[0-9]+' | head -n1)

exec /usr/bin/x11vnc -display ":$DISP" -auth "$AUTH" \
  -rfbport 5900 -rfbauth /home/robotpi/.vnc/passwd -forever -shared -noxdamage