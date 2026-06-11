#!/usr/bin/env bash
# Start RoboRun dashboard.
#   ./scripts/start.sh                      # webcam-only mode
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

# Use project venv if present, otherwise system python
if [[ -f .venv/bin/python3 ]]; then
  PYTHON=".venv/bin/python3"
else
  PYTHON="python3"
fi

# Kill existing instance on same port
PORT="${ROBORUN_PORT:-8765}"
if lsof -nP -iTCP:$PORT -sTCP:LISTEN >/dev/null 2>&1; then
  echo "▷ stopping existing RoboRun on :$PORT"
  pkill -f "roborun.server" 2>/dev/null || true
  sleep 1
fi

# Start RoboRun
echo "▷ starting RoboRun on http://127.0.0.1:$PORT"
$PYTHON -m roborun.server &
RR_PID=$!
sleep 2

if ! lsof -nP -iTCP:$PORT -sTCP:LISTEN >/dev/null 2>&1; then
  echo "✗ RoboRun failed to start" >&2
  exit 1
fi

echo "✓ RoboRun  pid=$RR_PID  http://127.0.0.1:$PORT"

cat <<EOF

  RoboRun:     http://127.0.0.1:$PORT
  Webcam:      Start from the UI (Vision tab or Control tab)

EOF
