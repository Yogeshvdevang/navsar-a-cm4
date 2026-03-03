#!/usr/bin/env python3
"""Read NMEA GPS data from serial and show live lat/lon graph in HTML."""

from __future__ import annotations

import argparse
import json
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import serial

DEFAULT_PORT = "/dev/ttyAMA5"
DEFAULT_BAUD_RATE = 230400
DEFAULT_FREQUENCY = 5.0
DEFAULT_HOST = "0.0.0.0"
DEFAULT_WEB_PORT = 8080
DEFAULT_INIT_WAIT_S = 5.0

HTML_PAGE = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>GPS Live Graph</title>
  <style>
    body {{ font-family: Arial, sans-serif; margin: 0; background: #f2f6fb; color: #0f1b2d; }}
    .wrap {{ max-width: 1000px; margin: 24px auto; padding: 0 16px; }}
    .card {{ background: #fff; border-radius: 10px; box-shadow: 0 8px 24px rgba(0,0,0,0.08); padding: 16px; }}
    h1 {{ margin: 0 0 8px 0; font-size: 22px; }}
    .meta {{ margin: 0 0 12px 0; color: #43536c; font-size: 14px; }}
    #status {{ margin: 0 0 12px 0; font-size: 14px; }}
    canvas {{ width: 100%; height: 520px; border: 1px solid #d9e2ef; border-radius: 8px; background: #fcfdff; }}
  </style>
</head>
<body>
  <div class="wrap">
    <div class="card">
      <h1>Real-time GPS Track</h1>
      <p class="meta">Serial: {serial_port} @ {baud_rate} baud | Refresh: {frequency} Hz</p>
      <p id="status">Waiting for GPS data...</p>
      <canvas id="plot" width="960" height="520"></canvas>
    </div>
  </div>

  <script>
    const pollMs = {poll_ms};
    const canvas = document.getElementById("plot");
    const ctx = canvas.getContext("2d");
    const statusEl = document.getElementById("status");

    function draw(points) {{
      const w = canvas.width;
      const h = canvas.height;
      const pad = 30;

      ctx.clearRect(0, 0, w, h);
      ctx.strokeStyle = "#dbe5f3";
      ctx.lineWidth = 1;
      for (let i = 0; i <= 8; i++) {{
        const y = pad + ((h - 2 * pad) * i / 8);
        ctx.beginPath();
        ctx.moveTo(pad, y);
        ctx.lineTo(w - pad, y);
        ctx.stroke();
      }}
      for (let i = 0; i <= 8; i++) {{
        const x = pad + ((w - 2 * pad) * i / 8);
        ctx.beginPath();
        ctx.moveTo(x, pad);
        ctx.lineTo(x, h - pad);
        ctx.stroke();
      }}

      if (!points.length) {{
        return;
      }}

      let minLat = points[0][0], maxLat = points[0][0];
      let minLon = points[0][1], maxLon = points[0][1];
      for (const [lat, lon] of points) {{
        if (lat < minLat) minLat = lat;
        if (lat > maxLat) maxLat = lat;
        if (lon < minLon) minLon = lon;
        if (lon > maxLon) maxLon = lon;
      }}

      const lonPad = Math.max((maxLon - minLon) * 0.1, 0.0002);
      const latPad = Math.max((maxLat - minLat) * 0.1, 0.0002);
      minLon -= lonPad; maxLon += lonPad;
      minLat -= latPad; maxLat += latPad;

      const toX = (lon) => pad + (lon - minLon) * (w - 2 * pad) / (maxLon - minLon || 1);
      const toY = (lat) => h - pad - (lat - minLat) * (h - 2 * pad) / (maxLat - minLat || 1);

      ctx.strokeStyle = "#0077cc";
      ctx.lineWidth = 2;
      ctx.beginPath();
      points.forEach(([lat, lon], i) => {{
        const x = toX(lon);
        const y = toY(lat);
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }});
      ctx.stroke();

      const last = points[points.length - 1];
      const lx = toX(last[1]);
      const ly = toY(last[0]);
      ctx.fillStyle = "#e63946";
      ctx.beginPath();
      ctx.arc(lx, ly, 5, 0, Math.PI * 2);
      ctx.fill();
    }}

    async function refresh() {{
      try {{
        const resp = await fetch("/data", {{ cache: "no-store" }});
        const data = await resp.json();
        draw(data.points);

        if (data.last) {{
          statusEl.textContent = `Points: ${{data.points.length}} | Lat: ${{data.last[0].toFixed(7)}} | Lon: ${{data.last[1].toFixed(7)}}`;
        }} else {{
          statusEl.textContent = "Waiting for GPS data...";
        }}
      }} catch (err) {{
        statusEl.textContent = "Connection error while fetching /data";
      }}
    }}

    refresh();
    setInterval(refresh, pollMs);
  </script>
</body>
</html>
"""


def nmea_to_decimal(raw: str, direction: str) -> float:
    """Convert NMEA ddmm.mmmm / dddmm.mmmm into signed decimal degrees."""
    if not raw or not direction:
        raise ValueError("Missing NMEA coordinate parts")

    value = float(raw)
    degrees = int(value // 100)
    minutes = value - (degrees * 100)
    decimal = degrees + (minutes / 60.0)
    if direction in ("S", "W"):
        decimal = -decimal
    return decimal


def parse_nmea_lat_lon(line: str) -> tuple[float, float] | None:
    """Parse lat/lon from NMEA sentence if available."""
    if not line.startswith("$"):
        return None
    parts = line.split(",")
    if len(parts) < 6:
        return None

    sentence = parts[0]
    try:
        if sentence.endswith("GGA"):
            if len(parts) > 6 and parts[6] == "0":
                return None
            return nmea_to_decimal(parts[2], parts[3]), nmea_to_decimal(parts[4], parts[5])
        if sentence.endswith("RMC"):
            if len(parts) > 2 and parts[2] != "A":
                return None
            return nmea_to_decimal(parts[3], parts[4]), nmea_to_decimal(parts[5], parts[6])
    except (ValueError, IndexError):
        return None
    return None


def serial_reader(
    ser: serial.Serial, points: deque[tuple[float, float]], lock: threading.Lock
) -> None:
    """Continuously read NMEA sentences and push valid points."""
    while True:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode("ascii", errors="ignore").strip()
        coord = parse_nmea_lat_lon(line)
        if coord is None:
            continue
        with lock:
            points.append(coord)


def make_handler(
    points: deque[tuple[float, float]],
    lock: threading.Lock,
    serial_port: str,
    baud_rate: int,
    frequency: float,
    poll_ms: int,
) -> type[BaseHTTPRequestHandler]:
    class Handler(BaseHTTPRequestHandler):
        def _send(self, status: int, content_type: str, body: bytes) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:
            path = urlparse(self.path).path
            if path in ("/", "/index.html"):
                html = HTML_PAGE.format(
                    serial_port=serial_port,
                    baud_rate=baud_rate,
                    frequency=frequency,
                    poll_ms=poll_ms,
                ).encode("utf-8")
                self._send(200, "text/html; charset=utf-8", html)
                return

            if path == "/data":
                with lock:
                    data = list(points)
                payload = {"points": data, "last": data[-1] if data else None}
                self._send(200, "application/json", json.dumps(payload).encode("utf-8"))
                return

            self._send(404, "text/plain; charset=utf-8", b"Not Found")

        def log_message(self, _format: str, *_args: object) -> None:
            return

    return Handler


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Read NMEA from serial and show live GPS graph in a browser."
    )
    parser.add_argument("--port", default=DEFAULT_PORT, help="Serial port device")
    parser.add_argument(
        "--baud-rate",
        "--baud",
        dest="baud_rate",
        type=int,
        default=DEFAULT_BAUD_RATE,
        help="Serial baudrate",
    )
    parser.add_argument(
        "--frequency",
        type=float,
        default=DEFAULT_FREQUENCY,
        help="Graph refresh frequency in Hz",
    )
    parser.add_argument(
        "--history",
        type=int,
        default=2000,
        help="Maximum number of recent points to keep",
    )
    parser.add_argument("--host", default=DEFAULT_HOST, help="Web server host")
    parser.add_argument("--web-port", type=int, default=DEFAULT_WEB_PORT, help="Web server port")
    parser.add_argument(
        "--init-wait-s",
        type=float,
        default=DEFAULT_INIT_WAIT_S,
        help="Initial wait (seconds) before starting NMEA reading",
    )
    args = parser.parse_args()

    if args.frequency <= 0:
        parser.error("--frequency must be greater than 0")
    if args.init_wait_s < 0:
        parser.error("--init-wait-s must be >= 0")
    poll_ms = int(1000.0 / args.frequency)

    ser = serial.Serial(args.port, args.baud_rate, timeout=1.0)
    points: deque[tuple[float, float]] = deque(maxlen=args.history)
    lock = threading.Lock()

    if args.init_wait_s > 0:
        print(f"Waiting {args.init_wait_s:.1f}s for GPS sensor warm-up...")
        time.sleep(args.init_wait_s)

    threading.Thread(target=serial_reader, args=(ser, points, lock), daemon=True).start()

    handler = make_handler(points, lock, args.port, args.baud_rate, args.frequency, poll_ms)
    server = ThreadingHTTPServer((args.host, args.web_port), handler)

    print(f"Serving GPS graph at http://{args.host}:{args.web_port}")
    print(f"Reading NMEA from {args.port} at {args.baud_rate} baud")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        ser.close()


if __name__ == "__main__":
    main()
