"""GPS Injector module. Provides gps injector utilities for NAVISAR."""

import argparse
import time
import random
from dataclasses import dataclass
from pathlib import Path

import serial
try:
    import yaml
except ModuleNotFoundError as exc:
    raise ModuleNotFoundError(
        "Missing dependency 'PyYAML' (module 'yaml'). "
        "Install with `pip install pyyaml` or `pip install -r requirements.txt`."
    ) from exc

from navisar.pixhawk.fake_gps_nmea import enu_to_gps, gga_sentence, rmc_sentence, speed_course_from_enu


@dataclass
class HomeLocation:
    """Home reference used for ENU-to-GPS conversion."""
    lat: float
    lon: float
    alt: float


def load_home(path):
    """Load home location from a YAML file."""
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    lat = data.get("lat")
    lon = data.get("lon")
    alt = data.get("alt")
    if lat is None or lon is None or alt is None:
        raise ValueError("Home file must define lat, lon, alt")
    return HomeLocation(float(lat), float(lon), float(alt))


def hdop_from_sats(sats):
    """Estimate HDOP from a satellite count heuristic."""
    if sats < 15:
        return 1.3
    if sats < 18:
        return 1.0
    return 0.7


class FakeSatellites:
    """Simple satellite-count simulator for NMEA output."""
    def __init__(self, min_sats=14, max_sats=20, update_s=7.0):
        """Initialize bounds and update cadence."""
        self.min_sats = min_sats
        self.max_sats = max_sats
        self.update_s = update_s
        self.sats = min_sats
        self.last_update = time.time()

    def update(self, ekf_ok=True):
        """Update and return the simulated satellite count."""
        now = time.time()
        if now - self.last_update < self.update_s:
            return self.sats
        self.last_update = now

        if not ekf_ok:
            self.sats = max(12, self.sats - 1)
            return self.sats

        if self.sats < 18:
            self.sats += 1
        else:
            self.sats += random.choice([-1, 0, 1])
            self.sats = max(18, min(self.max_sats, self.sats))

        if now % 60.0 < 5.0:
            self.sats = max(18, self.sats - 1)

        self.sats = max(self.min_sats, min(self.max_sats, self.sats))
        return self.sats


class NmeaGpsInjector:
    """Convert VPS odometry into NMEA serial output."""
    def __init__(
        self,
        port,
        baud,
        home,
        rate_hz=5.0,
        frame="enu",
        fix_quality=1,
        satellites=10,
        hdop=0.9,
    ):
        """Configure the injector and smoothing parameters."""
        self.port = port
        self.baud = baud
        self.home = home
        # Enforce 5-10 Hz update rate to keep GPS smooth and stable.
        self.rate_hz = min(max(rate_hz, 5.0), 10.0)
        self.frame = frame
        self.fix_quality = fix_quality
        self.satellites = satellites
        self.hdop = hdop
        self._ser = None
        self._last_send = 0.0
        self._last_pos = None
        self._last_time = None
        self._smooth_alpha = 0.2
        self._max_step_m = 1.5
        self._max_speed_delta_mps = 2.0
        self._max_heading_delta_deg = 20.0
        self._smoothed = None
        self._last_speed = 0.0
        self._last_course = 0.0
        self._fake_sats = FakeSatellites()

    def _open(self):
        """Open the serial connection."""
        self._ser = serial.Serial(self.port, self.baud, timeout=0)

    def _transform_to_enu(self, x, y, z, vx, vy, vz):
        """Convert input frame to ENU if needed."""
        if self.frame == "ned":
            x_e = y
            y_n = x
            z_u = -z
            vx_e = vy
            vy_n = vx
            vz_u = -vz
            return x_e, y_n, z_u, vx_e, vy_n, vz_u
            return x, y, z, vx, vy, vz

    def _smooth_state(self, x, y, z, vx, vy):
        """Apply smoothing and clamp sudden jumps in state."""
        if self._smoothed is None:
            self._smoothed = (x, y, z)
            return x, y, z, float(vx), float(vy)
        px, py, pz = self._smoothed
        nx = px + self._smooth_alpha * (x - px)
        ny = py + self._smooth_alpha * (y - py)
        nz = pz + self._smooth_alpha * (z - pz)
        dx = nx - px
        dy = ny - py
        dz = nz - pz
        step = float((dx * dx + dy * dy + dz * dz) ** 0.5)
        if self._max_step_m > 0.0 and step > self._max_step_m:
            scale = self._max_step_m / step
            nx = px + dx * scale
            ny = py + dy * scale
            nz = pz + dz * scale
        self._smoothed = (nx, ny, nz)
        vx_s = vx
        vy_s = vy
        speed = float((vx_s * vx_s + vy_s * vy_s) ** 0.5)
        if abs(speed - self._last_speed) > self._max_speed_delta_mps:
            if speed > 1e-3:
                scale = (self._last_speed + self._max_speed_delta_mps) / speed
                vx_s *= scale
                vy_s *= scale
                speed = float((vx_s * vx_s + vy_s * vy_s) ** 0.5)
        self._last_speed = speed
        return nx, ny, nz, vx_s, vy_s

    def _maybe_send(self, x, y, z, vx, vy, vz):
        """Send NMEA updates when the rate limit allows."""
        now = time.time()
        if now - self._last_send < (1.0 / self.rate_hz):
            return
        x_e, y_n, z_u, vx_e, vy_n, _vz_u = self._transform_to_enu(x, y, z, vx, vy, vz)
        x_e, y_n, z_u, vx_e, vy_n = self._smooth_state(x_e, y_n, z_u, vx_e, vy_n)
        lat, lon, alt = enu_to_gps(x_e, y_n, z_u, self.home.lat, self.home.lon, self.home.alt)
        speed_mps, course_deg = speed_course_from_enu(vx_e, vy_n)
        if speed_mps < 0.05:
            course_deg = self._last_course
        else:
            delta = (course_deg - self._last_course + 540.0) % 360.0 - 180.0
            if abs(delta) > self._max_heading_delta_deg:
                course_deg = (self._last_course + self._max_heading_delta_deg * (1 if delta > 0 else -1)) % 360.0
        self._last_course = course_deg
        sats = self._fake_sats.update(ekf_ok=True)
        hdop = hdop_from_sats(sats)
        gga = gga_sentence(
            lat,
            lon,
            alt,
            fix_quality=self.fix_quality,
            satellites=sats,
            hdop=hdop,
        )
        rmc = rmc_sentence(
            lat,
            lon,
            speed_mps,
            course_deg,
            status="A" if self.fix_quality > 0 else "V",
        )
        self._ser.write(gga.encode("ascii"))
        self._ser.write(rmc.encode("ascii"))
        self._last_send = now

    def run_from_vo(self):
        """Run the VO pipeline and forward updates as NMEA."""
        from navisar.main import build_vo_pipeline

        vo, _mavlink_interface = build_vo_pipeline()

        def on_update(x, y, z, dx_m, dy_m, dz_m, *_rest):
            now = time.time()
            if self._last_pos is None:
                self._last_pos = (x, y, z)
                self._last_time = now
                return
            dt = max(1e-3, now - self._last_time)
            vx = (x - self._last_pos[0]) / dt
            vy = (y - self._last_pos[1]) / dt
            vz = (z - self._last_pos[2]) / dt
            self._last_pos = (x, y, z)
            self._last_time = now
            self._maybe_send(x, y, z, vx, vy, vz)

        self._open()
        print(f"Sending NMEA on {self.port} @ {self.baud} ({self.rate_hz} Hz)")
        vo.run(on_update=on_update)


def _build_arg_parser():
    """Build the CLI argument parser."""
    parser = argparse.ArgumentParser(description="Inject VPS as fake GPS over NMEA.")
    parser.add_argument("--port", required=True, help="Serial port (e.g. /dev/ttyAMA0)")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--rate", type=float, default=5.0, help="NMEA send rate (Hz)")
    parser.add_argument("--home", default="data/home_locations/site_A.yaml", help="Home YAML file")
    parser.add_argument("--frame", choices=["enu", "ned"], default="enu", help="VPS frame")
    parser.add_argument("--fix-quality", type=int, default=1, help="NMEA fix quality")
    parser.add_argument("--satellites", type=int, default=10, help="NMEA satellites count")
    parser.add_argument("--hdop", type=float, default=0.9, help="NMEA HDOP")
    return parser


def main():
    """CLI entry point for the GPS injector."""
    args = _build_arg_parser().parse_args()
    home = load_home(args.home)
    injector = NmeaGpsInjector(
        port=args.port,
        baud=args.baud,
        home=home,
        rate_hz=args.rate,
        frame=args.frame,
        fix_quality=args.fix_quality,
        satellites=args.satellites,
        hdop=args.hdop,
    )
    injector.run_from_vo()


if __name__ == "__main__":
    main()
