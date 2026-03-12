#!/usr/bin/env python3
"""Isolated optical-flow path plotter with live HTML graph.

Reads MTF-01 optical-flow sensor data from serial, integrates motion from origin,
and serves a live XY graph in a browser.
"""

from __future__ import annotations

import json
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_DIR = REPO_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from navisar.sensors.optical_flow import MTF01OpticalFlowReader

DEFAULT_PORT = "/dev/ttyAMA3"
DEFAULT_BAUD = 115200
DEFAULT_HOST = "0.0.0.0"
DEFAULT_WEB_PORT = 8082
DEFAULT_POLL_HZ = 20.0
DEFAULT_HISTORY = 2000
DEFAULT_GAIN = 1.0
DEFAULT_EMA_ALPHA = 0.2
DEFAULT_DEADBAND = 0.0
DEFAULT_HEARTBEAT_INTERVAL_S = 0.6
DEFAULT_PRINT_EVERY = 1
DEFAULT_TERMINAL_MONITOR_HZ = 2.0

# Normal run config (no CLI needed). Edit here if required.
RUN_PORT = DEFAULT_PORT
RUN_BAUD = DEFAULT_BAUD
RUN_HOST = DEFAULT_HOST
RUN_WEB_PORT = DEFAULT_WEB_PORT
RUN_POLL_HZ = DEFAULT_POLL_HZ
RUN_HISTORY = DEFAULT_HISTORY
RUN_GAIN = DEFAULT_GAIN
RUN_EMA_ALPHA = DEFAULT_EMA_ALPHA
RUN_DEADBAND = DEFAULT_DEADBAND
RUN_SWAP_XY = False
RUN_INVERT_X = False
RUN_INVERT_Y = False
RUN_HEARTBEAT_INTERVAL_S = DEFAULT_HEARTBEAT_INTERVAL_S
RUN_PRINT_SAMPLES = True
RUN_PRINT_EVERY = DEFAULT_PRINT_EVERY
RUN_TERMINAL_MONITOR_HZ = DEFAULT_TERMINAL_MONITOR_HZ

HTML_PAGE = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Optical Flow Live Path</title>
  <style>
    body {{ margin: 0; font-family: Arial, sans-serif; background: #f4f6fb; color: #0f172a; }}
    .wrap {{ max-width: 1024px; margin: 18px auto; padding: 0 14px; }}
    .card {{ background: #fff; border-radius: 10px; box-shadow: 0 6px 20px rgba(0,0,0,.08); padding: 14px; }}
    h1 {{ margin: 0 0 8px 0; font-size: 22px; }}
    .meta {{ margin: 0 0 8px 0; color: #334155; font-size: 14px; }}
    .status {{ margin: 0 0 12px 0; color: #0f172a; font-size: 14px; }}
    .row {{ display: flex; align-items: center; gap: 10px; margin: 0 0 12px 0; }}
    button {{ border: 1px solid #1d4ed8; background: #2563eb; color: #fff; border-radius: 999px; padding: 8px 14px; cursor: pointer; }}
    button:hover {{ background: #1d4ed8; }}
    .axis-row {{ display: flex; flex-wrap: wrap; align-items: center; gap: 8px; margin: 0 0 10px 0; }}
    .axis-row button {{ border-color: #334155; background: #475569; }}
    .axis-row button.active {{ background: #059669; border-color: #047857; }}
    .axis-tag {{ font-size: 13px; color: #0f172a; }}
    canvas {{ width: 100%; height: 560px; border: 1px solid #d8e0ee; border-radius: 8px; background: #ffffff; }}
    .hint {{ margin-top: 8px; color: #475569; font-size: 13px; }}
  </style>
</head>
<body>
  <div class="wrap">
    <div class="card">
      <h1>Optical Flow Path From Origin</h1>
      <p class="meta">
        Serial: {serial_port} @ {baud_rate} | Refresh: {poll_hz} Hz
      </p>
      <div class="row">
        <button id="reset-btn" type="button">Reset Origin</button>
      </div>
      <div class="axis-row">
        <button id="swap-btn" type="button">Swap X/Y</button>
        <button id="invx-btn" type="button">Invert X</button>
        <button id="invy-btn" type="button">Invert Y</button>
        <button id="axis-default-btn" type="button">Axis Default</button>
        <span class="axis-tag" id="axis-status">Axis: swap=off invertX=off invertY=off</span>
      </div>
      <p class="status" id="status">Waiting for optical flow data...</p>
      <canvas id="plot" width="980" height="560"></canvas>
      <p class="hint">
        Move sensor left/right/up/down: path should move left/right/up/down on graph.
      </p>
    </div>
  </div>

  <script>
    const canvas = document.getElementById("plot");
    const ctx = canvas.getContext("2d");
    const statusEl = document.getElementById("status");
    const axisStatusEl = document.getElementById("axis-status");
    const resetBtn = document.getElementById("reset-btn");
    const swapBtn = document.getElementById("swap-btn");
    const invxBtn = document.getElementById("invx-btn");
    const invyBtn = document.getElementById("invy-btn");
    const axisDefaultBtn = document.getElementById("axis-default-btn");
    const pollMs = {poll_ms};

    function safeRange(min, max, minSpan) {{
      if (!Number.isFinite(min) || !Number.isFinite(max)) return [-1, 1];
      if (min === max) {{
        const pad = Math.max(minSpan, Math.abs(min) * 0.2, 1);
        return [min - pad, max + pad];
      }}
      const span = max - min;
      const pad = Math.max(span * 0.15, minSpan);
      return [min - pad, max + pad];
    }}

    function draw(points) {{
      const w = canvas.width;
      const h = canvas.height;
      const pad = 36;

      ctx.clearRect(0, 0, w, h);
      ctx.fillStyle = "#fff";
      ctx.fillRect(0, 0, w, h);

      if (!points.length) {{
        return;
      }}

      let minX = points[0][0], maxX = points[0][0];
      let minY = points[0][1], maxY = points[0][1];
      for (const [x, y] of points) {{
        if (x < minX) minX = x;
        if (x > maxX) maxX = x;
        if (y < minY) minY = y;
        if (y > maxY) maxY = y;
      }}
      if (0 < minX) minX = 0;
      if (0 > maxX) maxX = 0;
      if (0 < minY) minY = 0;
      if (0 > maxY) maxY = 0;

      const [x0, x1] = safeRange(minX, maxX, 1.0);
      const [y0, y1] = safeRange(minY, maxY, 1.0);

      const toPx = (x) => pad + ((x - x0) * (w - 2 * pad) / (x1 - x0 || 1));
      const toPy = (y) => h - pad - ((y - y0) * (h - 2 * pad) / (y1 - y0 || 1));

      ctx.strokeStyle = "#e2e8f0";
      ctx.lineWidth = 1;
      for (let i = 0; i <= 8; i += 1) {{
        const gy = pad + ((h - 2 * pad) * i / 8);
        const gx = pad + ((w - 2 * pad) * i / 8);
        ctx.beginPath(); ctx.moveTo(pad, gy); ctx.lineTo(w - pad, gy); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(gx, pad); ctx.lineTo(gx, h - pad); ctx.stroke();
      }}

      const ox = toPx(0);
      const oy = toPy(0);
      ctx.strokeStyle = "#94a3b8";
      ctx.lineWidth = 1.6;
      ctx.beginPath(); ctx.moveTo(pad, oy); ctx.lineTo(w - pad, oy); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(ox, pad); ctx.lineTo(ox, h - pad); ctx.stroke();

      ctx.fillStyle = "#334155";
      ctx.font = "12px sans-serif";
      ctx.fillText("X", w - pad + 6, oy - 2);
      ctx.fillText("Y", ox + 6, pad - 8);
      ctx.fillText("Origin (0,0)", ox + 8, oy - 8);

      ctx.strokeStyle = "#16a34a";
      ctx.lineWidth = 2.2;
      ctx.beginPath();
      points.forEach(([x, y], i) => {{
        const px = toPx(x);
        const py = toPy(y);
        if (i === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      }});
      ctx.stroke();

      const [lx, ly] = points[points.length - 1];
      const lpx = toPx(lx);
      const lpy = toPy(ly);
      ctx.fillStyle = "#ef4444";
      ctx.beginPath();
      ctx.arc(lpx, lpy, 5, 0, Math.PI * 2);
      ctx.fill();
    }}

    function setAxisUi(axis) {{
      const swap = !!axis.swap_xy;
      const invx = !!axis.invert_x;
      const invy = !!axis.invert_y;
      swapBtn.classList.toggle("active", swap);
      invxBtn.classList.toggle("active", invx);
      invyBtn.classList.toggle("active", invy);
      axisStatusEl.textContent = `Axis: swap=${{swap ? "on" : "off"}} invertX=${{invx ? "on" : "off"}} invertY=${{invy ? "on" : "off"}}`;
    }}

    async function postAxis(nextAxis) {{
      try {{
        const resp = await fetch("/axis", {{
          method: "POST",
          headers: {{ "Content-Type": "application/json" }},
          body: JSON.stringify(nextAxis),
        }});
        const payload = await resp.json();
        if (payload.axis) setAxisUi(payload.axis);
      }} catch (_err) {{}}
    }}

    async function refresh() {{
      try {{
        const resp = await fetch("/data", {{ cache: "no-store" }});
        const payload = await resp.json();
        draw(payload.points || []);
        setAxisUi(payload.axis || {{}});
        const latest = payload.latest || {{}};
        if (payload.points && payload.points.length) {{
          const pkt = latest.packet_count ?? 0;
          const err = latest.reader_error ? ` | error=${{latest.reader_error}}` : "";
          statusEl.textContent =
            `Points=${{payload.points.length}} | X=${{Number(latest.x || 0).toFixed(3)}} Y=${{Number(latest.y || 0).toFixed(3)}} | ` +
            `flow_vx=${{latest.flow_vx_mps ?? "--"}} flow_vy=${{latest.flow_vy_mps ?? "--"}} m/s ` +
            `dist=${{latest.distance_m ?? "--"}} m quality=${{latest.flow_q ?? "--"}} packets=${{pkt}}${{err}}`;
        }} else {{
          const err = latest.reader_error ? ` (error: ${{latest.reader_error}})` : "";
          statusEl.textContent = `Waiting for optical flow data...${{err}}`;
        }}
      }} catch (_err) {{
        statusEl.textContent = "Failed to fetch /data";
      }}
    }}

    resetBtn.addEventListener("click", async () => {{
      try {{
        await fetch("/reset", {{ method: "POST" }});
      }} catch (_err) {{}}
      refresh();
    }});

    swapBtn.addEventListener("click", async () => {{
      const active = swapBtn.classList.contains("active");
      await postAxis({{ swap_xy: !active }});
      refresh();
    }});
    invxBtn.addEventListener("click", async () => {{
      const active = invxBtn.classList.contains("active");
      await postAxis({{ invert_x: !active }});
      refresh();
    }});
    invyBtn.addEventListener("click", async () => {{
      const active = invyBtn.classList.contains("active");
      await postAxis({{ invert_y: !active }});
      refresh();
    }});
    axisDefaultBtn.addEventListener("click", async () => {{
      await postAxis({{ swap_xy: false, invert_x: false, invert_y: false }});
      refresh();
    }});

    refresh();
    setInterval(refresh, pollMs);
  </script>
</body>
</html>
"""

class SharedState:
    def __init__(self, max_points: int) -> None:
        self.lock = threading.Lock()
        self.points: deque[tuple[float, float]] = deque(maxlen=max_points)
        self.points.append((0.0, 0.0))
        self.swap_xy = bool(RUN_SWAP_XY)
        self.invert_x = bool(RUN_INVERT_X)
        self.invert_y = bool(RUN_INVERT_Y)
        self.x = 0.0
        self.y = 0.0
        self.last_time_ms: int | None = None
        self.flow_ema_x = 0.0
        self.flow_ema_y = 0.0
        self.latest: dict[str, float | int | None] = {
            "x": 0.0,
            "y": 0.0,
            "flow_vx": None,
            "flow_vy": None,
            "flow_vx_mps": None,
            "flow_vy_mps": None,
            "flow_q": None,
            "flow_ok": None,
            "distance_mm": None,
            "distance_m": None,
            "packet_count": 0,
            "raw_bytes": 0,
            "last_sample_time_s": None,
            "reader_error": None,
        }

    def reset_origin(self) -> None:
        with self.lock:
            self.points.clear()
            self.points.append((0.0, 0.0))
            self.x = 0.0
            self.y = 0.0
            self.last_time_ms = None
            self.flow_ema_x = 0.0
            self.flow_ema_y = 0.0
            self.latest["x"] = 0.0
            self.latest["y"] = 0.0
            self.latest["reader_error"] = None


def apply_deadband(v: float, threshold: float) -> float:
    if abs(v) < threshold:
        return 0.0
    return v


def optical_reader_loop(
    state: SharedState,
    port: str,
    baud_rate: int,
    read_hz: float,
    gain: float,
    ema_alpha: float,
    deadband: float,
    heartbeat_interval_s: float,
    print_samples: bool,
    print_every: int,
) -> None:
    packet_count = 0
    last_sample_time_ms = None
    loop_sleep_s = 1.0 / max(5.0, float(read_hz))
    reader = MTF01OpticalFlowReader(
        port=port,
        baudrate=int(baud_rate),
        data_frequency=max(1.0, float(read_hz)),
        heartbeat_interval_s=float(heartbeat_interval_s),
        print_enabled=False,
        on_sample=None,
    )
    try:
        reader.start()
        while True:
            sample = reader.get_latest()
            err = reader.get_last_error()
            if err is not None:
                with state.lock:
                    state.latest["reader_error"] = str(err)
                time.sleep(loop_sleep_s)
                continue

            if sample is None:
                time.sleep(loop_sleep_s)
                continue

            now_ms = int(getattr(sample, "time_ms", 0) or 0)
            if last_sample_time_ms is not None and now_ms == last_sample_time_ms:
                time.sleep(loop_sleep_s)
                continue
            last_sample_time_ms = now_ms
            packet_count += 1

            with state.lock:
                if state.last_time_ms is None:
                    dt = 1.0 / max(1.0, float(read_hz))
                else:
                    dt = (now_ms - state.last_time_ms) / 1000.0
                    if dt <= 0.0 or dt > 0.3:
                        dt = 1.0 / max(1.0, float(read_hz))
                state.last_time_ms = now_ms

                # Sensor provides flow_vx/flow_vy directly in m/s.
                fx = float(getattr(sample, "flow_vx", 0.0))
                fy = float(getattr(sample, "flow_vy", 0.0))

                if state.swap_xy:
                    fx, fy = fy, fx
                if state.invert_x:
                    fx = -fx
                if state.invert_y:
                    fy = -fy

                fx = apply_deadband(fx, deadband)
                fy = apply_deadband(fy, deadband)

                state.flow_ema_x = (ema_alpha * fx) + ((1.0 - ema_alpha) * state.flow_ema_x)
                state.flow_ema_y = (ema_alpha * fy) + ((1.0 - ema_alpha) * state.flow_ema_y)

                state.x += state.flow_ema_x * dt * gain
                state.y += state.flow_ema_y * dt * gain
                state.points.append((state.x, state.y))

                state.latest = {
                    "x": state.x,
                    "y": state.y,
                    "flow_vx": getattr(sample, "flow_vx", None),
                    "flow_vy": getattr(sample, "flow_vy", None),
                    "flow_vx_mps": getattr(sample, "flow_vx", None),
                    "flow_vy_mps": getattr(sample, "flow_vy", None),
                    "flow_q": getattr(sample, "flow_quality", None),
                    "flow_ok": getattr(sample, "flow_ok", None),
                    "distance_mm": getattr(sample, "distance_mm", None),
                    "distance_m": (
                        (float(getattr(sample, "distance_mm")) / 1000.0)
                        if getattr(sample, "distance_mm", None) is not None
                        else None
                    ),
                    "packet_count": packet_count,
                    "raw_bytes": None,
                    "last_sample_time_s": time.time(),
                    "reader_error": None,
                }

                if print_samples and packet_count % max(1, int(print_every)) == 0:
                    ts = time.strftime("%H:%M:%S")
                    print(
                        f"[{ts}] [{packet_count}] flow_vx={state.latest['flow_vx']} flow_vy={state.latest['flow_vy']} "
                        f"vx_mps={state.latest['flow_vx_mps']} vy_mps={state.latest['flow_vy_mps']} "
                        f"q={state.latest['flow_q']} ok={state.latest['flow_ok']} dist_m={state.latest['distance_m']} "
                        f"x={state.x:.3f} y={state.y:.3f}",
                        flush=True,
                    )
            time.sleep(loop_sleep_s)
    except Exception as exc:
        with state.lock:
            state.latest["reader_error"] = str(exc)
        raise
    finally:
        try:
            reader.stop()
        except Exception:
            pass


def make_handler(
    state: SharedState,
    serial_port: str,
    baud_rate: int,
    poll_hz: float,
) -> type[BaseHTTPRequestHandler]:
    poll_ms = max(20, int(1000.0 / max(1e-6, poll_hz)))

    class Handler(BaseHTTPRequestHandler):
        def _send(self, code: int, content_type: str, body: bytes) -> None:
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:
            path = urlparse(self.path).path
            if path in ("/", "/index.html"):
                body = HTML_PAGE.format(
                    serial_port=serial_port,
                    baud_rate=baud_rate,
                    poll_hz=poll_hz,
                    poll_ms=poll_ms,
                ).encode("utf-8")
                self._send(200, "text/html; charset=utf-8", body)
                return

            if path == "/data":
                with state.lock:
                    payload = {
                        "points": list(state.points),
                        "latest": dict(state.latest),
                        "axis": {
                            "swap_xy": state.swap_xy,
                            "invert_x": state.invert_x,
                            "invert_y": state.invert_y,
                        },
                    }
                body = json.dumps(payload).encode("utf-8")
                self._send(200, "application/json", body)
                return

            self._send(404, "text/plain; charset=utf-8", b"Not Found")

        def do_POST(self) -> None:
            path = urlparse(self.path).path
            if path == "/reset":
                state.reset_origin()
                body = b'{"ok": true}'
                self._send(200, "application/json", body)
                return
            if path == "/axis":
                length = int(self.headers.get("Content-Length", "0") or 0)
                raw = self.rfile.read(length) if length > 0 else b"{}"
                try:
                    req = json.loads(raw.decode("utf-8"))
                except json.JSONDecodeError:
                    req = {}
                with state.lock:
                    if "swap_xy" in req:
                        state.swap_xy = bool(req.get("swap_xy"))
                    if "invert_x" in req:
                        state.invert_x = bool(req.get("invert_x"))
                    if "invert_y" in req:
                        state.invert_y = bool(req.get("invert_y"))
                    payload = {
                        "ok": True,
                        "axis": {
                            "swap_xy": state.swap_xy,
                            "invert_x": state.invert_x,
                            "invert_y": state.invert_y,
                        },
                    }
                body = json.dumps(payload).encode("utf-8")
                self._send(200, "application/json", body)
                return
            self._send(404, "text/plain; charset=utf-8", b"Not Found")

        def log_message(self, _fmt: str, *_args: object) -> None:
            return

    return Handler


def main() -> None:
    alpha = min(1.0, max(0.0, float(RUN_EMA_ALPHA)))
    state = SharedState(max_points=max(20, int(RUN_HISTORY)))

    reader = threading.Thread(
        target=optical_reader_loop,
        kwargs={
            "state": state,
            "port": RUN_PORT,
            "baud_rate": int(RUN_BAUD),
            "read_hz": float(RUN_POLL_HZ),
            "gain": float(RUN_GAIN),
            "ema_alpha": alpha,
            "deadband": float(RUN_DEADBAND),
            "heartbeat_interval_s": float(RUN_HEARTBEAT_INTERVAL_S),
            "print_samples": bool(RUN_PRINT_SAMPLES),
            "print_every": int(RUN_PRINT_EVERY),
        },
        daemon=True,
    )
    reader.start()

    handler = make_handler(
        state=state,
        serial_port=RUN_PORT,
        baud_rate=int(RUN_BAUD),
        poll_hz=float(RUN_POLL_HZ),
    )
    server = ThreadingHTTPServer((RUN_HOST, int(RUN_WEB_PORT)), handler)
    url = f"http://127.0.0.1:{int(RUN_WEB_PORT)}/"
    print(f"Optical flow graph server started: {url}")
    print("Press Ctrl+C to stop.")
    monitor_interval_s = 1.0 / max(0.1, float(RUN_TERMINAL_MONITOR_HZ))

    def terminal_monitor() -> None:
        while True:
            with state.lock:
                latest = dict(state.latest)
            ts = time.strftime("%H:%M:%S")
            pkt = int(latest.get("packet_count") or 0)
            raw_bytes = int(latest.get("raw_bytes") or 0)
            flow_vx = latest.get("flow_vx")
            flow_vy = latest.get("flow_vy")
            flow_q = latest.get("flow_q")
            flow_ok = latest.get("flow_ok")
            x = float(latest.get("x") or 0.0)
            y = float(latest.get("y") or 0.0)
            last_sample_time_s = latest.get("last_sample_time_s")
            reader_error = latest.get("reader_error")
            if isinstance(last_sample_time_s, (int, float)):
                age_s = max(0.0, time.time() - float(last_sample_time_s))
                age_text = f"{age_s:.2f}s"
            else:
                age_text = "n/a"
            print(
                f"[{ts}] live raw_bytes={raw_bytes} packets={pkt} "
                f"flow_vx={flow_vx} flow_vy={flow_vy} q={flow_q} ok={flow_ok} "
                f"x={x:.3f} y={y:.3f} sample_age={age_text}"
                + (f" error={reader_error}" if reader_error else ""),
                flush=True,
            )
            time.sleep(monitor_interval_s)

    monitor = threading.Thread(target=terminal_monitor, daemon=True)
    monitor.start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()


if __name__ == "__main__":
    main()

"""
optical_gps_tuning:
    lat_scale: 2.0
    lon_scale: 0.5
    alt_offset_m: 0.0
    vo_scale: 2.0
    vo_lat_scale: 2.0
    vo_lon_scale: 0.5
    """