#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DISPLAY_NUMBER="${DISPLAY_NUMBER:-99}"
export DISPLAY=":$DISPLAY_NUMBER"
VNC_PORT="${VNC_PORT:-5900}"
WEB_PORT="${WEB_PORT:-6080}"
NOVNC_LISTEN="${NOVNC_LISTEN:-127.0.0.1}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"

cleanup() {
  jobs -pr | xargs -r kill 2>/dev/null || true
}
trap cleanup EXIT INT TERM

Xvfb "$DISPLAY" -screen 0 1920x1080x24 -ac +extension GLX +render -noreset &
sleep 1
fluxbox >/tmp/baja_fluxbox.log 2>&1 &
x11vnc -display "$DISPLAY" -forever -shared -nopw -rfbport "$VNC_PORT" >/tmp/baja_x11vnc.log 2>&1 &
websockify --web=/usr/share/novnc "$NOVNC_LISTEN:$WEB_PORT" "localhost:$VNC_PORT" >/tmp/baja_novnc.log 2>&1 &

echo "RViz is listening on $NOVNC_LISTEN:$WEB_PORT"
echo "SSH tunnel: ssh -L $WEB_PORT:localhost:$WEB_PORT user@server"
echo "Then open: http://localhost:$WEB_PORT/vnc.html"
"$SCRIPT_DIR/run.sh" --headless-gazebo "$@"
