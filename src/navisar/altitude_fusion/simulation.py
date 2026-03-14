"""Synthetic scenario harness for altitude fusion validation."""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional

from .fusion import AltitudeFusion, FusionConfig, FusionInput, LidarSample, MavSample


@dataclass
class SimRow:
    t: float
    true_amsl_m: float
    true_ground_m: float
    baro_rel_m: float
    lidar_dist_m: Optional[float]
    roll_rad: float
    pitch_rad: float


def _rows_flat_takeoff_landing(dt: float = 1.0 / 30.0) -> Iterable[SimRow]:
    t = 0.0
    home_amsl = 950.0
    ground = 948.0
    while t <= 20.0:
        if t < 5.0:
            agl = (t / 5.0) * 8.0
        elif t < 15.0:
            agl = 8.0
        else:
            agl = max(0.0, 8.0 * (1.0 - (t - 15.0) / 5.0))
        amsl = ground + agl
        yield SimRow(t, amsl, ground, amsl - home_amsl, agl, 0.01, -0.01)
        t += dt


def _rows_terrace_hover(dt: float = 1.0 / 30.0) -> Iterable[SimRow]:
    t = 0.0
    home_amsl = 952.0
    while t <= 16.0:
        if t < 8.0:
            ground = 948.0
        else:
            ground = 949.0
        agl = 3.0
        amsl = ground + agl
        yield SimRow(t, amsl, ground, amsl - home_amsl, agl, 0.02, 0.01)
        t += dt


def _rows_terrace_edge_drop(dt: float = 1.0 / 30.0) -> Iterable[SimRow]:
    t = 0.0
    home_amsl = 952.0
    while t <= 12.0:
        ground = 948.0 if t < 6.0 else 943.0
        amsl = 951.0
        agl = amsl - ground
        yield SimRow(t, amsl, ground, amsl - home_amsl, agl, 0.01, -0.01)
        t += dt


def _rows_lidar_dropout(dt: float = 1.0 / 30.0) -> Iterable[SimRow]:
    t = 0.0
    home_amsl = 950.0
    ground = 947.5
    while t <= 12.0:
        amsl = 948.0 + 0.4 * t
        agl = amsl - ground
        lidar = agl if not (4.0 <= t <= 8.0) else None
        yield SimRow(t, amsl, ground, amsl - home_amsl, lidar, 0.01, 0.01)
        t += dt


def _rows_large_tilt_reject(dt: float = 1.0 / 30.0) -> Iterable[SimRow]:
    t = 0.0
    home_amsl = 951.0
    ground = 948.0
    while t <= 10.0:
        amsl = 952.0
        agl = amsl - ground
        roll = math.radians(30.0) if 4.0 <= t <= 6.0 else math.radians(3.0)
        pitch = math.radians(2.0)
        yield SimRow(t, amsl, ground, amsl - home_amsl, agl, roll, pitch)
        t += dt


SCENARIOS = {
    "flat_takeoff_landing": _rows_flat_takeoff_landing,
    "terrace_hover": _rows_terrace_hover,
    "terrace_edge_drop": _rows_terrace_edge_drop,
    "lidar_dropout": _rows_lidar_dropout,
    "large_tilt_reject": _rows_large_tilt_reject,
}


def run_scenario(name: str, cfg: Optional[FusionConfig] = None) -> List[dict]:
    if name not in SCENARIOS:
        raise ValueError(f"Unknown scenario: {name}")
    fusion = AltitudeFusion(cfg or FusionConfig())
    rows_out = []
    lat = 12.0
    lon = 77.0
    t0 = 1_700_000_000.0

    for row in SCENARIOS[name]():
        ts = t0 + row.t
        mav = MavSample(
            timestamp_s=ts,
            roll_rad=row.roll_rad,
            pitch_rad=row.pitch_rad,
            yaw_rad=0.0,
            alt_amsl_m=row.true_amsl_m,
            relative_alt_m=row.baro_rel_m,
            lat_deg=lat,
            lon_deg=lon,
        )
        lidar = None
        if row.lidar_dist_m is not None:
            lidar = LidarSample(
                timestamp_s=ts,
                distance_m=row.lidar_dist_m,
                quality=80.0,
                healthy=True,
            )

        out = fusion.step(FusionInput(mav=mav, lidar=lidar))
        rows_out.append(
            {
                "t": row.t,
                "scenario": name,
                "true_amsl_m": row.true_amsl_m,
                "true_ground_m": row.true_ground_m,
                "baro_rel_m": row.baro_rel_m,
                "lidar_dist_m": row.lidar_dist_m,
                "fused_amsl_m": out.h_amsl_est_m,
                "pred_amsl_m": out.h_amsl_pred_m,
                "ground_est_m": out.h_ground_est_m,
                "agl_filt_m": out.h_agl_filt_m,
                "mode": out.mode,
                "lidar_valid": out.lidar_valid,
                "lidar_conf": out.lidar_confidence,
            }
        )

    return rows_out


def run_all() -> List[dict]:
    rows = []
    for name in SCENARIOS:
        rows.extend(run_scenario(name))
    return rows


def write_csv(rows: List[dict], out_path: Path) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    with out_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser(description="Run altitude fusion synthetic scenarios")
    parser.add_argument(
        "--scenario",
        default="all",
        choices=["all", *SCENARIOS.keys()],
        help="Scenario to run",
    )
    parser.add_argument(
        "--out",
        default="data/altitude_fusion_sim.csv",
        help="Output CSV path",
    )
    args = parser.parse_args()

    if args.scenario == "all":
        rows = run_all()
    else:
        rows = run_scenario(args.scenario)
    write_csv(rows, Path(args.out))
    print(f"Wrote {len(rows)} rows -> {args.out}")


if __name__ == "__main__":
    main()
