#!/usr/bin/env python3
"""Isolated optical-flow path plotter with live HTML graph.

Reads MTF-01 optical-flow sensor data from serial, integrates motion from origin,
and serves a live XY graph in a browser.
"""

from __future__ import annotations

import json
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import serial

MICOLINK_HEAD = 0xEF
MICOLINK_MSG_ID_RANGE_SENSOR = 0x51
MICOLINK_MAX_PAYLOAD_LEN = 64

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
RUN_INVERT_X = False
RUN_INVERT_Y = False
RUN_HEARTBEAT_INTERVAL_S = DEFAULT_HEARTBEAT_INTERVAL_S
RUN_PRINT_SAMPLES = True
RUN_PRINT_EVERY = DEFAULT_PRINT_EVERY

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
    const resetBtn = document.getElementById("reset-btn");
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

    async function refresh() {{
      try {{
        const resp = await fetch("/data", {{ cache: "no-store" }});
        const payload = await resp.json();
        draw(payload.points || []);
        const latest = payload.latest || {{}};
        if (payload.points && payload.points.length) {{
          const pkt = latest.packet_count ?? 0;
          const err = latest.reader_error ? ` | error=${{latest.reader_error}}` : "";
          statusEl.textContent =
            `Points=${{payload.points.length}} | X=${{Number(latest.x || 0).toFixed(3)}} Y=${{Number(latest.y || 0).toFixed(3)}} | ` +
            `flow_vx=${{latest.flow_vx ?? "--"}} flow_vy=${{latest.flow_vy ?? "--"}} quality=${{latest.flow_q ?? "--"}} packets=${{pkt}}${{err}}`;
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

    refresh();
    setInterval(refresh, pollMs);
  </script>
</body>
</html>
"""


@dataclass
class OpticalSample:
    time_ms: int
    flow_vx: int
    flow_vy: int
    flow_q: int
    flow_ok: int
    distance_mm: int


def create_heartbeat(seq: int) -> bytes:
    """Build one Micolink heartbeat message to trigger sensor streaming."""
    time_ms = int(time.time() * 1000) & 0xFFFFFFFF
    msg = bytearray([0xEF, 0x01, 0x00, 0x01, seq & 0xFF, 0x0D])
    msg.extend(struct.pack("<I", time_ms))
    msg.extend(bytes([0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00]))
    msg.append(sum(msg) & 0xFF)
    return bytes(msg)


class MicolinkParser:
    """Byte-wise parser for Micolink messages."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._status = 0
        self._hdr = [0, 0, 0, 0, 0, 0]
        self._payload = bytearray(MICOLINK_MAX_PAYLOAD_LEN)
        self._payload_cnt = 0

    @staticmethod
    def _checksum(head: list[int], payload: bytes) -> int:
        return (sum(head) + sum(payload)) & 0xFF

    def feed(self, b: int) -> OpticalSample | None:
        if self._status == 0:
            if b == MICOLINK_HEAD:
                self._hdr[0] = b
                self._status = 1
            return None

        if 1 <= self._status <= 5:
            self._hdr[self._status] = b
            self._status += 1
            if self._status == 6:
                payload_len = self._hdr[5]
                if payload_len > MICOLINK_MAX_PAYLOAD_LEN:
                    self.reset()
            return None

        payload_len = self._hdr[5]
        if self._status == 6:
            if payload_len == 0:
                self._status = 7
                return self.feed(b)
            self._payload[self._payload_cnt] = b
            self._payload_cnt += 1
            if self._payload_cnt >= payload_len:
                self._status = 7
            return None

        if self._status == 7:
            payload = bytes(self._payload[:payload_len])
            calc = self._checksum(self._hdr, payload)
            msg_id = self._hdr[3]
            checksum_ok = calc == b
            self.reset()
            if not checksum_ok or msg_id != MICOLINK_MSG_ID_RANGE_SENSOR or payload_len < 24:
                return None
            return decode_optical_payload(payload)

        self.reset()
        return None


def decode_optical_payload(payload: bytes) -> OpticalSample:
    data = struct.unpack("<IIBBBBhhBBH", payload[:24])
    return OpticalSample(
        time_ms=int(data[0]),
        distance_mm=int(data[1]),
        flow_vx=int(data[6]),
        flow_vy=int(data[7]),
        flow_q=int(data[8]),
        flow_ok=1 if int(data[9]) == 1 else 0,
    )


class SharedState:
    def __init__(self, max_points: int) -> None:
        self.lock = threading.Lock()
        self.points: deque[tuple[float, float]] = deque(maxlen=max_points)
        self.points.append((0.0, 0.0))
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
            "flow_q": None,
            "flow_ok": None,
            "distance_mm": None,
            "packet_count": 0,
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


def serial_loop(
    state: SharedState,
    port: str,
    baud_rate: int,
    gain: float,
    ema_alpha: float,
    deadband: float,
    invert_x: bool,
    invert_y: bool,
    heartbeat_interval_s: float,
    print_samples: bool,
    print_every: int,
) -> None:
    parser = MicolinkParser()
    seq = 0
    packet_count = 0
    hb_interval = max(0.2, float(heartbeat_interval_s))
    try:
        with serial.Serial(
            port,
            baud_rate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1.0,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        ) as ser:
            ser.setDTR(True)
            ser.setRTS(True)
            time.sleep(0.1)
            ser.reset_input_buffer()
            ser.reset_output_buffer()

            for _ in range(3):
                ser.write(create_heartbeat(seq))
                ser.flush()
                seq = (seq + 1) & 0xFF
                time.sleep(0.1)
            time.sleep(0.3)
            ser.reset_input_buffer()

            last_hb = time.time()
            while True:
                now = time.time()
                if (now - last_hb) >= hb_interval:
                    ser.write(create_heartbeat(seq))
                    ser.flush()
                    seq = (seq + 1) & 0xFF
                    last_hb = now

                raw = ser.read(256)
                if not raw:
                    continue
                for b in raw:
                    sample = parser.feed(b)
                    if sample is None:
                        continue
                    packet_count += 1
                    if print_samples and packet_count % max(1, int(print_every)) == 0:
                        ts = time.strftime("%H:%M:%S")
                        print(
                            f"[{ts}] [{packet_count}] flow_vx={sample.flow_vx} flow_vy={sample.flow_vy} "
                            f"q={sample.flow_q} ok={sample.flow_ok} dist_mm={sample.distance_mm} "
                            f"x={state.x:.3f} y={state.y:.3f}",
                            flush=True,
                        )

                    now_ms = sample.time_ms
                    with state.lock:
                        if state.last_time_ms is None:
                            dt = 1.0 / 60.0
                        else:
                            dt = (now_ms - state.last_time_ms) / 1000.0
                            if dt <= 0.0 or dt > 0.3:
                                dt = 1.0 / 60.0
                        state.last_time_ms = now_ms

                        fx = float(sample.flow_vx)
                        fy = float(sample.flow_vy)
                        if invert_x:
                            fx = -fx
                        if invert_y:
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
                            "flow_vx": sample.flow_vx,
                            "flow_vy": sample.flow_vy,
                            "flow_q": sample.flow_q,
                            "flow_ok": sample.flow_ok,
                            "distance_mm": sample.distance_mm,
                            "packet_count": packet_count,
                            "reader_error": None,
                        }
    except Exception as exc:
        with state.lock:
            state.latest["reader_error"] = str(exc)
        raise


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
            self._send(404, "text/plain; charset=utf-8", b"Not Found")

        def log_message(self, _fmt: str, *_args: object) -> None:
            return

    return Handler


def main() -> None:
    alpha = min(1.0, max(0.0, float(RUN_EMA_ALPHA)))
    state = SharedState(max_points=max(20, int(RUN_HISTORY)))

    reader = threading.Thread(
        target=serial_loop,
        kwargs={
            "state": state,
            "port": RUN_PORT,
            "baud_rate": int(RUN_BAUD),
            "gain": float(RUN_GAIN),
            "ema_alpha": alpha,
            "deadband": float(RUN_DEADBAND),
            "invert_x": bool(RUN_INVERT_X),
            "invert_y": bool(RUN_INVERT_Y),
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
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()


if __name__ == "__main__":
    main()
