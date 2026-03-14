"""Threaded lidar input for altitude fusion service."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Optional

import serial

from .fusion import LidarSample


@dataclass
class LidarInputConfig:
    """Serial lidar input configuration."""

    port: str = "/dev/ttyUSB0"
    baud: int = 115200
    timeout_s: float = 0.1
    reconnect_delay_s: float = 1.0
    min_quality: float = 0.0


class LidarInput:
    """Continuously reads lidar lines in 'distance_m[,quality]' format."""

    def __init__(self, config: Optional[LidarInputConfig] = None):
        self.cfg = config or LidarInputConfig()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._latest: Optional[LidarSample] = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="lidar-input", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def latest(self) -> Optional[LidarSample]:
        with self._lock:
            return self._latest

    def _parse_line(self, line: str) -> Optional[LidarSample]:
        line = line.strip()
        if not line:
            return None
        parts = [p.strip() for p in line.split(",")]
        if not parts:
            return None
        try:
            distance_m = float(parts[0])
        except ValueError:
            return None
        quality = None
        if len(parts) > 1 and parts[1] != "":
            try:
                quality = float(parts[1])
            except ValueError:
                quality = None
        healthy = True
        if quality is not None and quality < self.cfg.min_quality:
            healthy = False
        return LidarSample(
            timestamp_s=time.time(),
            distance_m=distance_m,
            quality=quality,
            healthy=healthy,
        )

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                with serial.Serial(
                    self.cfg.port,
                    self.cfg.baud,
                    timeout=self.cfg.timeout_s,
                ) as ser:
                    self._run_connected(ser)
            except Exception as exc:
                print(f"[lidar_input] reconnecting after error: {exc}")
                time.sleep(self.cfg.reconnect_delay_s)

    def _run_connected(self, ser) -> None:
        while not self._stop.is_set():
            raw = ser.readline()
            if not raw:
                continue
            try:
                line = raw.decode("ascii", errors="ignore")
            except Exception:
                continue
            sample = self._parse_line(line)
            if sample is None:
                continue
            with self._lock:
                self._latest = sample
