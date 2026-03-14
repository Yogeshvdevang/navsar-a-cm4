"""Threaded MAVLink input for altitude fusion service."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Optional

from pymavlink import mavutil

from .fusion import MavSample


@dataclass
class MavlinkInputConfig:
    """MAVLink input configuration."""

    device: str = "/dev/ttyACM0"
    baud: int = 115200
    source_system: int = 200
    source_component: int = 193
    reconnect_delay_s: float = 1.0
    heartbeat_timeout_s: float = 5.0
    attitude_rate_hz: float = 30.0
    global_position_rate_hz: float = 20.0


class MavlinkInput:
    """Continuously reads ATTITUDE and GLOBAL_POSITION_INT from Pixhawk."""

    def __init__(self, config: Optional[MavlinkInputConfig] = None):
        self.cfg = config or MavlinkInputConfig()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

        self._master = None
        self._latest: Optional[MavSample] = None
        self._last_att = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        self._last_global = {
            "alt_amsl_m": 0.0,
            "relative_alt_m": 0.0,
            "lat_deg": None,
            "lon_deg": None,
        }

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="mavlink-input", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def latest(self) -> Optional[MavSample]:
        with self._lock:
            return self._latest

    def _connect(self):
        master = mavutil.mavlink_connection(
            self.cfg.device,
            baud=self.cfg.baud,
            source_system=int(self.cfg.source_system),
            source_component=int(self.cfg.source_component),
        )
        master.wait_heartbeat(timeout=self.cfg.heartbeat_timeout_s)
        self._request_rate(master, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, self.cfg.attitude_rate_hz)
        self._request_rate(
            master,
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
            self.cfg.global_position_rate_hz,
        )
        return master

    @staticmethod
    def _request_rate(master, msg_id: int, rate_hz: float) -> None:
        if rate_hz <= 0:
            return
        interval_us = int(1_000_000.0 / float(rate_hz))
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            interval_us,
            0,
            0,
            0,
            0,
            0,
        )

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                self._master = self._connect()
                self._run_connected()
            except Exception as exc:
                print(f"[mavlink_input] reconnecting after error: {exc}")
                time.sleep(self.cfg.reconnect_delay_s)

    def _run_connected(self) -> None:
        while not self._stop.is_set():
            msg = self._master.recv_match(blocking=True, timeout=0.2)
            if msg is None:
                continue

            msg_type = msg.get_type()
            now = time.time()

            if msg_type == "ATTITUDE":
                self._last_att["roll"] = float(msg.roll)
                self._last_att["pitch"] = float(msg.pitch)
                self._last_att["yaw"] = float(msg.yaw)
            elif msg_type == "GLOBAL_POSITION_INT":
                self._last_global["alt_amsl_m"] = float(msg.alt) / 1000.0
                self._last_global["relative_alt_m"] = float(msg.relative_alt) / 1000.0
                self._last_global["lat_deg"] = float(msg.lat) / 1e7
                self._last_global["lon_deg"] = float(msg.lon) / 1e7
            else:
                continue

            sample = MavSample(
                timestamp_s=now,
                roll_rad=self._last_att["roll"],
                pitch_rad=self._last_att["pitch"],
                yaw_rad=self._last_att["yaw"],
                alt_amsl_m=self._last_global["alt_amsl_m"],
                relative_alt_m=self._last_global["relative_alt_m"],
                lat_deg=self._last_global["lat_deg"],
                lon_deg=self._last_global["lon_deg"],
            )
            with self._lock:
                self._latest = sample
