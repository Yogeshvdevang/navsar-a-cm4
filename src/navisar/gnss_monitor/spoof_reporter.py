"""Log and forward GNSS spoofing events to GCS."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from pathlib import Path

from pymavlink import mavutil


@dataclass
class SpoofReportConfig:
    """Configuration for spoofing event reporting."""

    log_path: str | None = None
    gcs_severity: str = "warning"
    min_interval_s: float = 2.0


class SpoofReporter:
    """Record spoofing events to disk and optionally send to GCS."""

    def __init__(self, config: SpoofReportConfig, mavlink_interface=None) -> None:
        self.config = config
        self.mavlink_interface = mavlink_interface
        self._last_sent_time = 0.0

    def report(self, reason, gps_local=None, drift_m=None, gps_fix_type=None, timestamp=None):
        """Log a spoofing event and forward it to GCS."""
        if timestamp is None:
            timestamp = time.time()
        self._log_event(reason, gps_local, drift_m, gps_fix_type, timestamp)
        self._send_gcs(reason, timestamp)

    def _log_event(self, reason, gps_local, drift_m, gps_fix_type, timestamp):
        if not self.config.log_path:
            return
        path = Path(self.config.log_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "timestamp": float(timestamp),
            "reason": str(reason),
            "gps_local": gps_local,
            "drift_m": drift_m,
            "gps_fix_type": gps_fix_type,
        }
        with path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(payload) + "\n")

    def _send_gcs(self, reason, timestamp):
        if self.mavlink_interface is None:
            return
        if self.config.min_interval_s > 0:
            if timestamp - self._last_sent_time < self.config.min_interval_s:
                return
        severity = self._severity_to_mavlink(self.config.gcs_severity)
        message = f"GPS spoofing detected: {reason}"
        self.mavlink_interface.send_statustext(message, severity=severity)
        self._last_sent_time = timestamp

    @staticmethod
    def _severity_to_mavlink(level):
        if not level:
            return mavutil.mavlink.MAV_SEVERITY_WARNING
        level = str(level).strip().lower()
        mapping = {
            "debug": mavutil.mavlink.MAV_SEVERITY_DEBUG,
            "info": mavutil.mavlink.MAV_SEVERITY_INFO,
            "notice": mavutil.mavlink.MAV_SEVERITY_NOTICE,
            "warning": mavutil.mavlink.MAV_SEVERITY_WARNING,
            "error": mavutil.mavlink.MAV_SEVERITY_ERROR,
            "critical": mavutil.mavlink.MAV_SEVERITY_CRITICAL,
            "alert": mavutil.mavlink.MAV_SEVERITY_ALERT,
            "emergency": mavutil.mavlink.MAV_SEVERITY_EMERGENCY,
        }
        return mapping.get(level, mavutil.mavlink.MAV_SEVERITY_WARNING)
