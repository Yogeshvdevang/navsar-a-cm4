#!/usr/bin/env python3
"""Isolated optical-flow path plotter with live HTML graph.

Reads MTF-01 optical-flow sensor data from serial, integrates motion from origin,
and serves a live XY graph in a browser.
"""

from __future__ import annotations

import json
import math
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
from navisar.sensors.compass import CompassReader

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
DEFAULT_COMPASS_HZ = 20.0
DEFAULT_HEADING_SMOOTH_ALPHA = 0.25

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
RUN_COMPASS_ENABLED = True
RUN_COMPASS_BUS = 1
RUN_COMPASS_HZ = DEFAULT_COMPASS_HZ
RUN_HEADING_SMOOTH_ALPHA = DEFAULT_HEADING_SMOOTH_ALPHA
RUN_ORIGIN_LAT_DEG = 0.0
RUN_ORIGIN_LON_DEG = 0.0

EARTH_RADIUS_M = 6378137.0

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
    .legend {{ margin-top: 6px; color: #334155; font-size: 13px; }}
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
      <p class="legend">
        Arrow legend: Blue = world velocity vector, Red = heading/yaw direction, Black = current endpoint direction marker.
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

    function drawArrow(x, y, angleRad, len, tailBack, shaftLen, wing, shaftColor, headColor, lineWidth) {{
      const dx = Math.sin(angleRad);
      const dy = -Math.cos(angleRad);
      const tipX = x + dx * len;
      const tipY = y + dy * len;
      const tailX = x - dx * tailBack;
      const tailY = y - dy * tailBack;
      const headBaseX = tipX - dx * shaftLen;
      const headBaseY = tipY - dy * shaftLen;
      const nx = -dy;
      const ny = dx;

      ctx.strokeStyle = shaftColor;
      ctx.lineWidth = lineWidth;
      ctx.beginPath();
      ctx.moveTo(tailX, tailY);
      ctx.lineTo(headBaseX, headBaseY);
      ctx.stroke();

      ctx.fillStyle = headColor;
      ctx.beginPath();
      ctx.moveTo(tipX, tipY);
      ctx.lineTo(headBaseX + nx * wing, headBaseY + ny * wing);
      ctx.lineTo(headBaseX - nx * wing, headBaseY - ny * wing);
      ctx.closePath();
      ctx.fill();
    }}

      function draw(points, headingDeg, vxWorldMps, vyWorldMps) {{
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
      ctx.fillText("Longitude East (+X)", w - pad - 130, oy - 8);
      ctx.fillText("Latitude North (+Y)", ox + 6, pad - 8);
      ctx.fillText("Home Lat/Lon (0,0)", ox + 8, oy - 8);

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
      const hasHeading = Number.isFinite(Number(headingDeg));
      // 3D-like velocity vector anchored at endpoint using world-frame vx/vy.
      const vx = Number(vxWorldMps);
      const vy = Number(vyWorldMps);
      const hasVel = Number.isFinite(vx) && Number.isFinite(vy);
      if (hasVel) {{
        const maxVelRef = 1.0;
        const normX = Math.max(-1, Math.min(1, vx / maxVelRef));
        const normY = Math.max(-1, Math.min(1, vy / maxVelRef));
        const tipX = lpx + normX * (w - 2 * pad) * 0.12;
        const tipY = lpy - normY * (h - 2 * pad) * 0.12;
        const dirX = tipX - lpx;
        const dirY = tipY - lpy;
        const mag = Math.hypot(dirX, dirY);
        if (mag > 1e-6) {{
          const ux = dirX / mag;
          const uy = dirY / mag;
          const nx = -uy;
          const ny = ux;
          const head = 11;
          const wing = 6;
          const baseX = tipX - ux * head;
          const baseY = tipY - uy * head;

          ctx.strokeStyle = "rgba(15,23,42,0.20)";
          ctx.lineWidth = 8;
          ctx.beginPath();
          ctx.moveTo(lpx + 2, lpy + 2);
          ctx.lineTo(baseX + 2, baseY + 2);
          ctx.stroke();

          ctx.strokeStyle = "#2563eb";
          ctx.lineWidth = 5;
          ctx.beginPath();
          ctx.moveTo(lpx, lpy);
          ctx.lineTo(baseX, baseY);
          ctx.stroke();

          ctx.fillStyle = "#1d4ed8";
          ctx.beginPath();
          ctx.moveTo(tipX, tipY);
          ctx.lineTo(baseX + nx * wing, baseY + ny * wing);
          ctx.lineTo(baseX - nx * wing, baseY - ny * wing);
          ctx.closePath();
          ctx.fill();
        }}
      }}

      // Heading arrow from endpoint (drone z-rotation direction).
      if (hasHeading) {{
        const headingRad = Number(headingDeg) * Math.PI / 180.0;
        drawArrow(lpx, lpy, headingRad, 26, 8, 10, 4, "#dc2626", "#ef4444", 2.5);
      }}

      // Endpoint arrow marker (replace dot).
      const markerHeadingDeg = hasHeading
        ? Number(headingDeg)
        : (hasVel ? ((Math.atan2(vx, vy) * 180.0 / Math.PI + 360.0) % 360.0) : null);
      if (Number.isFinite(markerHeadingDeg)) {{
        const a = Number(markerHeadingDeg) * Math.PI / 180.0;
        drawArrow(lpx, lpy, a, 16, 6, 7, 3, "#030712", "#111827", 2.2);
      }} else {{
        ctx.fillStyle = "#111827";
        ctx.beginPath();
        ctx.arc(lpx, lpy, 3, 0, Math.PI * 2);
        ctx.fill();
      }}
      const hdgText = hasHeading ? Number(headingDeg).toFixed(1) : "--";
      const vxText = hasVel ? vx.toFixed(3) : "--";
      const vyText = hasVel ? vy.toFixed(3) : "--";
      ctx.fillStyle = "#0f172a";
      ctx.font = "12px monospace";
      ctx.fillText(`vx=${{vxText}} m/s  vy=${{vyText}} m/s  zrot=${{hdgText}} deg`, lpx + 10, lpy - 12);
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
        const latest = payload.latest || {{}};
        draw(payload.points || [], latest.heading_plot_deg, latest.vx_world_mps, latest.vy_world_mps);
        setAxisUi(payload.axis || {{}});
        if (payload.points && payload.points.length) {{
          const pkt = latest.packet_count ?? 0;
          const err = latest.reader_error ? ` | error=${{latest.reader_error}}` : "";
          const zrot = Number.isFinite(Number(latest.heading_plot_deg))
            ? Number(latest.heading_plot_deg).toFixed(1)
            : "--";
          const chdg = Number.isFinite(Number(latest.compass_heading_deg))
            ? Number(latest.compass_heading_deg).toFixed(1)
            : "--";
          const hsrc = latest.heading_source || "none";
          statusEl.textContent =
            `Points=${{payload.points.length}} | Lon(East)=${{Number(latest.x || 0).toFixed(3)}} m Lat(North)=${{Number(latest.y || 0).toFixed(3)}} m | ` +
            `final_lat=${{Number(latest.final_lat_deg || 0).toFixed(7)}} final_lon=${{Number(latest.final_lon_deg || 0).toFixed(7)}} | ` +
            `flow_vx=${{latest.flow_vx_mps ?? "--"}} flow_vy=${{latest.flow_vy_mps ?? "--"}} m/s ` +
            `zrot=${{zrot}} deg src=${{hsrc}} compass=${{chdg}} deg dist=${{latest.distance_m ?? "--"}} m quality=${{latest.flow_q ?? "--"}} packets=${{pkt}}${{err}}`;
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
        self.origin_lat_deg = float(RUN_ORIGIN_LAT_DEG)
        self.origin_lon_deg = float(RUN_ORIGIN_LON_DEG)
        self.swap_xy = bool(RUN_SWAP_XY)
        self.invert_x = bool(RUN_INVERT_X)
        self.invert_y = bool(RUN_INVERT_Y)
        self.x = 0.0
        self.y = 0.0
        self.final_lat_deg = self.origin_lat_deg
        self.final_lon_deg = self.origin_lon_deg
        self.last_time_ms: int | None = None
        self.flow_ema_x = 0.0
        self.flow_ema_y = 0.0
        self.fused_heading_deg: float | None = None
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
            "compass_heading_deg": None,
            "heading_plot_deg": None,
            "heading_source": "none",
            "vx_world_mps": 0.0,
            "vy_world_mps": 0.0,
            "origin_lat_deg": self.origin_lat_deg,
            "origin_lon_deg": self.origin_lon_deg,
            "final_lat_deg": self.final_lat_deg,
            "final_lon_deg": self.final_lon_deg,
            "packet_count": 0,
            "raw_bytes": 0,
            "last_sample_time_s": None,
            "reader_error": None,
            "compass_error": None,
        }

    def reset_origin(self) -> None:
        with self.lock:
            self.points.clear()
            self.points.append((0.0, 0.0))
            self.x = 0.0
            self.y = 0.0
            self.final_lat_deg = self.origin_lat_deg
            self.final_lon_deg = self.origin_lon_deg
            self.last_time_ms = None
            self.flow_ema_x = 0.0
            self.flow_ema_y = 0.0
            self.fused_heading_deg = None
            self.latest["x"] = 0.0
            self.latest["y"] = 0.0
            self.latest["origin_lat_deg"] = self.origin_lat_deg
            self.latest["origin_lon_deg"] = self.origin_lon_deg
            self.latest["final_lat_deg"] = self.final_lat_deg
            self.latest["final_lon_deg"] = self.final_lon_deg
            self.latest["reader_error"] = None


def apply_deadband(v: float, threshold: float) -> float:
    if abs(v) < threshold:
        return 0.0
    return v


def wrap_heading(deg: float) -> float:
    return (float(deg) + 360.0) % 360.0


def blend_heading(base_deg: float, target_deg: float, alpha: float) -> float:
    b = wrap_heading(base_deg)
    t = wrap_heading(target_deg)
    d = (t - b + 540.0) % 360.0 - 180.0
    return wrap_heading(b + float(alpha) * d)


def meters_to_lat_lon(
    origin_lat_deg: float,
    origin_lon_deg: float,
    east_m: float,
    north_m: float,
) -> tuple[float, float]:
    lat_rad = math.radians(float(origin_lat_deg))
    dlat_deg = math.degrees(float(north_m) / EARTH_RADIUS_M)
    cos_lat = math.cos(lat_rad)
    if abs(cos_lat) < 1e-9:
        cos_lat = 1e-9 if cos_lat >= 0.0 else -1e-9
    dlon_deg = math.degrees(float(east_m) / (EARTH_RADIUS_M * cos_lat))
    final_lat = float(origin_lat_deg) + dlat_deg
    final_lon = float(origin_lon_deg) + dlon_deg
    return final_lat, final_lon


def compass_loop(state: SharedState, preferred_bus: int, hz: float) -> None:
    sleep_s = 1.0 / max(1.0, float(hz))
    reader = None
    try:
        reader = CompassReader(preferred_bus=preferred_bus)
        while True:
            try:
                heading, _mg = reader.read_milligauss()
                with state.lock:
                    state.latest["compass_heading_deg"] = wrap_heading(float(heading))
                    state.latest["compass_error"] = None
            except Exception as exc:
                with state.lock:
                    state.latest["compass_error"] = str(exc)
            time.sleep(sleep_s)
    except Exception as exc:
        with state.lock:
            state.latest["compass_error"] = str(exc)
        while True:
            time.sleep(1.0)
    finally:
        if reader is not None:
            try:
                reader.close()
            except Exception:
                pass




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

                flow_vx_mps = float(getattr(sample, "flow_vx", 0.0))
                flow_vy_mps = float(getattr(sample, "flow_vy", 0.0))
                vx_body = flow_vx_mps
                vy_body = flow_vy_mps

                if state.swap_xy:
                    vx_body, vy_body = vy_body, vx_body
                if state.invert_x:
                    vx_body = -vx_body
                if state.invert_y:
                    vy_body = -vy_body

                vx_body = apply_deadband(vx_body, deadband)
                vy_body = apply_deadband(vy_body, deadband)

                # Required transform:
                # psi = pi/2 - heading_rad
                # [vx_world, vy_world]^T = R(psi) @ [vx_body, vy_body]^T
                compass_heading = state.latest.get("compass_heading_deg")
                if compass_heading is not None and math.isfinite(float(compass_heading)):
                    heading_rad = math.radians(float(compass_heading))
                    psi = math.pi / 2.0 - heading_rad
                    c = math.cos(psi)
                    s = math.sin(psi)
                    vx_world = c * vx_body - s * vy_body
                    vy_world = s * vx_body + c * vy_body
                    heading_source = "compass_rot"
                else:
                    # Fallback if compass missing: treat body frame as world frame.
                    vx_world = vx_body
                    vy_world = vy_body
                    heading_source = "flow_only"

                state.flow_ema_x = (ema_alpha * vx_world) + ((1.0 - ema_alpha) * state.flow_ema_x)
                state.flow_ema_y = (ema_alpha * vy_world) + ((1.0 - ema_alpha) * state.flow_ema_y)

                state.x += state.flow_ema_x * dt * gain
                state.y += state.flow_ema_y * dt * gain
                state.points.append((state.x, state.y))
                state.final_lat_deg, state.final_lon_deg = meters_to_lat_lon(
                    state.origin_lat_deg,
                    state.origin_lon_deg,
                    state.x,
                    state.y,
                )

                target_heading = None
                heading_fusion_source = "hold"
                if compass_heading is not None and math.isfinite(float(compass_heading)):
                    target_heading = wrap_heading(float(compass_heading))
                    heading_fusion_source = "compass"

                if target_heading is not None:
                    if state.fused_heading_deg is None:
                        state.fused_heading_deg = target_heading
                    else:
                        state.fused_heading_deg = blend_heading(
                            state.fused_heading_deg,
                            target_heading,
                            RUN_HEADING_SMOOTH_ALPHA,
                        )

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
                    "compass_heading_deg": (
                        float(compass_heading)
                        if compass_heading is not None and math.isfinite(float(compass_heading))
                        else None
                    ),
                    "heading_plot_deg": state.fused_heading_deg,
                    "heading_source": f"{heading_source}/{heading_fusion_source}",
                    "vx_world_mps": state.flow_ema_x,
                    "vy_world_mps": state.flow_ema_y,
                    "origin_lat_deg": state.origin_lat_deg,
                    "origin_lon_deg": state.origin_lon_deg,
                    "final_lat_deg": state.final_lat_deg,
                    "final_lon_deg": state.final_lon_deg,
                    "packet_count": packet_count,
                    "raw_bytes": None,
                    "last_sample_time_s": time.time(),
                    "reader_error": None,
                    "compass_error": state.latest.get("compass_error"),
                }

                if print_samples and packet_count % max(1, int(print_every)) == 0:
                    ts = time.strftime("%H:%M:%S")
                    print(
                        f"[{ts}] [{packet_count}] flow_vx={state.latest['flow_vx']} flow_vy={state.latest['flow_vy']} "
                        f"vx_mps={state.latest['flow_vx_mps']} vy_mps={state.latest['flow_vy_mps']} "
                        f"q={state.latest['flow_q']} ok={state.latest['flow_ok']} dist_m={state.latest['distance_m']} "
                        f"x={state.x:.3f} y={state.y:.3f} "
                        f"lat={state.final_lat_deg:.7f} lon={state.final_lon_deg:.7f}",
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
            try:
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
            except Exception as exc:
                body = json.dumps({"ok": False, "error": str(exc)}).encode("utf-8")
                self._send(500, "application/json", body)

        def do_POST(self) -> None:
            try:
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
            except Exception as exc:
                body = json.dumps({"ok": False, "error": str(exc)}).encode("utf-8")
                self._send(500, "application/json", body)

        def log_message(self, _fmt: str, *_args: object) -> None:
            return

    return Handler


def main() -> None:
    alpha = min(1.0, max(0.0, float(RUN_EMA_ALPHA)))
    state = SharedState(max_points=max(20, int(RUN_HISTORY)))

    if RUN_COMPASS_ENABLED:
        compass_thread = threading.Thread(
            target=compass_loop,
            kwargs={
                "state": state,
                "preferred_bus": int(RUN_COMPASS_BUS),
                "hz": float(RUN_COMPASS_HZ),
            },
            daemon=True,
        )
        compass_thread.start()

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
    preferred_port = int(RUN_WEB_PORT)
    server = None
    bound_port = None
    for port_try in range(preferred_port, preferred_port + 20):
        try:
            server = ThreadingHTTPServer((RUN_HOST, port_try), handler)
            bound_port = port_try
            break
        except OSError:
            continue
    if server is None or bound_port is None:
        raise RuntimeError(
            f"Failed to bind web server on {RUN_HOST} ports {preferred_port}-{preferred_port + 19}."
        )
    url = f"http://127.0.0.1:{bound_port}/"
    print(f"Optical flow graph server started: {url}")
    if bound_port != preferred_port:
        print(
            f"Note: preferred port {preferred_port} was busy; using {bound_port} instead."
        )
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
            compass_heading = latest.get("compass_heading_deg")
            x = float(latest.get("x") or 0.0)
            y = float(latest.get("y") or 0.0)
            final_lat = latest.get("final_lat_deg")
            final_lon = latest.get("final_lon_deg")
            last_sample_time_s = latest.get("last_sample_time_s")
            reader_error = latest.get("reader_error")
            compass_error = latest.get("compass_error")
            if isinstance(last_sample_time_s, (int, float)):
                age_s = max(0.0, time.time() - float(last_sample_time_s))
                age_text = f"{age_s:.2f}s"
            else:
                age_text = "n/a"
            print(
                f"[{ts}] live raw_bytes={raw_bytes} packets={pkt} "
                f"flow_vx={flow_vx} flow_vy={flow_vy} zrot={latest.get('heading_plot_deg')} "
                f"compass={compass_heading} src={latest.get('heading_source')} q={flow_q} ok={flow_ok} "
                f"x={x:.3f} y={y:.3f} lat={final_lat} lon={final_lon} sample_age={age_text}"
                + (f" error={reader_error}" if reader_error else "")
                + (f" compass_error={compass_error}" if compass_error else ""),
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
