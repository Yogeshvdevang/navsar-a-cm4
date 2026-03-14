"""Altitude fusion service runner for Raspberry Pi companion computer."""

from __future__ import annotations

import json
import signal
import threading
import time
from dataclasses import asdict
from pathlib import Path
from typing import Any, Dict, Optional

import yaml

from .fusion import AltitudeFusion, FusionConfig, FusionInput
from .gps_output import GpsOutput, GpsOutputConfig
from .lidar_input import LidarInput, LidarInputConfig
from .mavlink_input import MavlinkInput, MavlinkInputConfig


class AltitudeFusionService:
    """Coordinates MAVLink RX, lidar RX, fusion, and GPS output tasks."""

    def __init__(self, config_path: str = "config/altitude_fusion.yaml"):
        self.config_path = Path(config_path)
        self._cfg = self._load_config(self.config_path)

        fusion_cfg = FusionConfig(**self._cfg.get("fusion", {}))
        self._loop_hz = float(self._cfg.get("fusion", {}).get("loop_hz", 30.0))
        self._loop_period_s = 1.0 / max(1e-3, self._loop_hz)

        self._mav = MavlinkInput(MavlinkInputConfig(**self._cfg.get("mavlink_input", {})))
        self._lidar = LidarInput(LidarInputConfig(**self._cfg.get("lidar_input", {})))
        self._fusion = AltitudeFusion(fusion_cfg)
        self._gps_output = GpsOutput(GpsOutputConfig(**self._cfg.get("gps_output", {})))

        self._stop = threading.Event()
        self._log_path = Path(self._cfg.get("logging", {}).get("path", "data/altitude_fusion.log"))
        self._log_path.parent.mkdir(parents=True, exist_ok=True)
        self._print_interval_s = float(self._cfg.get("logging", {}).get("print_interval_s", 0.5))

    @staticmethod
    def _load_config(path: Path) -> Dict[str, Any]:
        with path.open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
        if not isinstance(data, dict):
            raise ValueError("Top-level config must be a mapping")
        return data

    def _register_signal_handlers(self):
        def _handler(_signum, _frame):
            self._stop.set()

        signal.signal(signal.SIGINT, _handler)
        signal.signal(signal.SIGTERM, _handler)

    def run(self) -> None:
        self._register_signal_handlers()
        self._mav.start()
        self._lidar.start()

        last_print = 0.0
        with self._log_path.open("a", encoding="utf-8") as log_handle:
            while not self._stop.is_set():
                cycle_start = time.time()

                mav = self._mav.latest()
                lidar = self._lidar.latest()
                if mav is None:
                    time.sleep(0.02)
                    continue

                out = self._fusion.step(FusionInput(mav=mav, lidar=lidar))
                gps_alt_m = self._fusion.gps_output_altitude_m()

                self._gps_output.send(
                    now_s=cycle_start,
                    lat_deg=mav.lat_deg,
                    lon_deg=mav.lon_deg,
                    alt_amsl_m=gps_alt_m,
                )

                record = {
                    "t": cycle_start,
                    "mode": out.mode,
                    "baro_rel_filt_m": out.baro_rel_filt_m,
                    "lidar_agl_filt_m": out.h_agl_filt_m,
                    "ground_est_m": out.h_ground_est_m,
                    "h_amsl_pred_m": out.h_amsl_pred_m,
                    "h_amsl_est_m": out.h_amsl_est_m,
                    "gps_alt_out_m": gps_alt_m,
                    "lidar_valid": out.lidar_valid,
                    "lidar_conf": out.lidar_confidence,
                    "k": out.correction_gain,
                    "gamma": out.ground_gamma,
                    "innovation_m": out.innovation_m,
                    "flags": out.validity_flags,
                }
                log_handle.write(json.dumps(record, separators=(",", ":")) + "\n")

                if cycle_start - last_print >= self._print_interval_s:
                    last_print = cycle_start
                    print(
                        "[alt_fusion] "
                        f"mode={out.mode} amsl={out.h_amsl_est_m:.3f} "
                        f"pred={out.h_amsl_pred_m:.3f} ground={out.h_ground_est_m:.3f} "
                        f"agl={0.0 if out.h_agl_filt_m is None else out.h_agl_filt_m:.3f} "
                        f"valid={out.lidar_valid} conf={out.lidar_confidence:.2f}"
                    )

                elapsed = time.time() - cycle_start
                sleep_s = self._loop_period_s - elapsed
                if sleep_s > 0.0:
                    time.sleep(sleep_s)

        self.close()

    def close(self) -> None:
        self._mav.stop()
        self._lidar.stop()
        self._gps_output.close()


def main() -> None:
    service = AltitudeFusionService()
    service.run()


if __name__ == "__main__":
    main()
