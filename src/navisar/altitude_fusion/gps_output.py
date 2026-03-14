"""AMSL output drivers for Pixhawk ingestion (NMEA serial or MAVLink GPS_INPUT)."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

from pymavlink import mavutil

from navisar.pixhawk.gps_output import NmeaSerialEmitter


@dataclass
class GpsOutputConfig:
    """Configuration for output path."""

    mode: str = "nmea_serial"  # nmea_serial | ardupilot_gps_input
    rate_hz: float = 10.0

    nmea_port: str = "/dev/ttyAMA0"
    nmea_baud: int = 230400
    nmea_fix_quality: int = 1
    nmea_min_sats: int = 10
    nmea_max_sats: int = 18
    nmea_update_s: float = 7.0

    mav_device: str = "/dev/ttyACM0"
    mav_baud: int = 115200
    gps_id: int = 0


class GpsOutput:
    """Rate-limited output of fused AMSL altitude."""

    def __init__(self, config: Optional[GpsOutputConfig] = None):
        self.cfg = config or GpsOutputConfig()
        self._period_s = 1.0 / max(1e-3, self.cfg.rate_hz)
        self._last_send_t = 0.0

        self._nmea = None
        self._master = None

        if self.cfg.mode == "nmea_serial":
            self._nmea = NmeaSerialEmitter(
                port=self.cfg.nmea_port,
                baud=self.cfg.nmea_baud,
                rate_hz=self.cfg.rate_hz,
                fix_quality=self.cfg.nmea_fix_quality,
                min_sats=self.cfg.nmea_min_sats,
                max_sats=self.cfg.nmea_max_sats,
                update_s=self.cfg.nmea_update_s,
                raw_print=False,
            )
        elif self.cfg.mode == "ardupilot_gps_input":
            self._master = mavutil.mavlink_connection(
                self.cfg.mav_device,
                baud=self.cfg.mav_baud,
                source_system=210,
                source_component=191,
            )
            self._master.wait_heartbeat(timeout=5.0)
        else:
            raise ValueError(f"Unsupported GPS output mode: {self.cfg.mode}")

    def ready(self, now_s: float) -> bool:
        return (now_s - self._last_send_t) >= self._period_s

    def send(
        self,
        *,
        now_s: float,
        lat_deg: Optional[float],
        lon_deg: Optional[float],
        alt_amsl_m: float,
        vn_mps: float = 0.0,
        ve_mps: float = 0.0,
        vd_mps: float = 0.0,
    ) -> None:
        """Send one fused altitude sample over configured output path."""
        if not self.ready(now_s):
            return

        if lat_deg is None:
            lat_deg = 0.0
        if lon_deg is None:
            lon_deg = 0.0

        if self.cfg.mode == "nmea_serial":
            self._nmea.send(
                lat=lat_deg,
                lon=lon_deg,
                alt_m=alt_amsl_m,
                vx_e=ve_mps,
                vy_n=vn_mps,
                ekf_ok=True,
                include_heading=True,
            )
        else:
            self._master.mav.gps_input_send(
                int(now_s * 1_000_000),
                self.cfg.gps_id,
                0,
                0,
                0,
                0,
                int(lat_deg * 1e7),
                int(lon_deg * 1e7),
                float(alt_amsl_m),
                1.0,
                1.0,
                float(vn_mps),
                float(ve_mps),
                float(vd_mps),
                0.5,
                1.0,
                10,
            )

        self._last_send_t = now_s

    def close(self) -> None:
        if self._nmea is not None:
            try:
                self._nmea._ser.close()  # pylint: disable=protected-access
            except Exception:
                pass
        if self._master is not None:
            try:
                self._master.close()
            except Exception:
                pass
