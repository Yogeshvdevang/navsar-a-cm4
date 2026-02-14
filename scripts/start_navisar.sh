#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.."; pwd)"
PYTHON_BIN="python3"
if [ -x "$ROOT_DIR/venv/bin/python" ]; then
  PYTHON_BIN="$ROOT_DIR/venv/bin/python"
fi
LOCK_FILE="$ROOT_DIR/.navisar.lock"

wait_for_display() {
  local display_name="${DISPLAY:-:0}"
  local display_num="${display_name#:}"
  display_num="${display_num%%.*}"
  local x_socket="/tmp/.X11-unix/X${display_num}"
  local xauth="${XAUTHORITY:-$HOME/.Xauthority}"
  local timeout_s="${NAVISAR_DISPLAY_WAIT_SECONDS:-120}"
  local waited=0

  while [ "$waited" -lt "$timeout_s" ]; do
    if [ -S "$x_socket" ] && [ -f "$xauth" ]; then
      return 0
    fi
    sleep 2
    waited=$((waited + 2))
  done
  echo "NAVISAR: display not ready after ${timeout_s}s (DISPLAY=${display_name}, XAUTHORITY=${xauth})." >&2
  return 1
}

if [ "${NAVISAR_REQUIRE_DISPLAY:-0}" = "1" ]; then
  wait_for_display
fi

exec /usr/bin/flock -n "$LOCK_FILE" /usr/bin/env PYTHONPATH="$ROOT_DIR/src" "$PYTHON_BIN" -m navisar.main
