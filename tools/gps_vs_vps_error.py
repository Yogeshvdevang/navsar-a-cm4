"""Fit affine calibration between raw GPS and derived GPS tracks."""

from __future__ import annotations

import argparse
import csv
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np


RAW_COLUMNS = {
    "altitude_m": "raw_gps_input_alt_m",
    "latitude": "raw_gps_input_lat",
    "longitude": "raw_gps_input_lon",
}

SENSOR_COLUMN_GROUPS = {
    "optical_flow": {
        "altitude_m": "syn_outputs_optical_flow_gps_port_six_parameters_altitude_m",
        "latitude": "syn_outputs_optical_flow_gps_port_six_parameters_latitude",
        "longitude": "syn_outputs_optical_flow_gps_port_six_parameters_longitude",
    },
    "vo": {
        "altitude_m": "syn_outputs_vo_gps_port_six_parameters_altitude_m",
        "latitude": "syn_outputs_vo_gps_port_six_parameters_latitude",
        "longitude": "syn_outputs_vo_gps_port_six_parameters_longitude",
    },
}


@dataclass(frozen=True)
class FitResult:
    scale: float
    offset: float
    r2: float
    rmse: float
    mae: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Fit per-axis affine calibration between raw GPS columns and "
            "optical-flow / VO derived GPS columns."
        )
    )
    parser.add_argument(
        "input_path",
        nargs="?",
        default="simulation/gps-op.csv",
        help="Path to the input TSV/CSV file.",
    )
    parser.add_argument(
        "--output",
        default="simulation/gps-op-calibrated.csv",
        help="Where to write the calibrated table.",
    )
    parser.add_argument(
        "--report-json",
        default="simulation/gps-op-calibration-report.json",
        help="Where to write the calibration report as JSON.",
    )
    return parser.parse_args()


def detect_dialect(input_path: Path) -> csv.Dialect:
    sample = input_path.read_text(encoding="utf-8", errors="replace")[:4096]
    try:
        return csv.Sniffer().sniff(sample, delimiters=",\t;")
    except csv.Error:
        return csv.excel_tab


def read_rows(input_path: Path, dialect: csv.Dialect) -> list[dict[str, str]]:
    with input_path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle, dialect=dialect))


def numeric_column(rows: Iterable[dict[str, str]], column: str) -> np.ndarray:
    return np.array([float(row[column]) for row in rows], dtype=float)


def fit_affine(source: np.ndarray, target: np.ndarray) -> tuple[FitResult, np.ndarray]:
    design = np.vstack([source, np.ones(len(source), dtype=float)]).T
    scale, offset = np.linalg.lstsq(design, target, rcond=None)[0]
    corrected = scale * source + offset
    residuals = target - corrected
    ss_res = float(np.dot(residuals, residuals))
    centered = target - float(target.mean())
    ss_tot = float(np.dot(centered, centered))
    r2 = 1.0 if ss_tot == 0.0 else 1.0 - ss_res / ss_tot
    rmse = float(np.sqrt(np.mean(residuals**2)))
    mae = float(np.mean(np.abs(residuals)))
    return FitResult(float(scale), float(offset), r2, rmse, mae), corrected


def apply_calibration(
    rows: list[dict[str, str]],
) -> tuple[list[dict[str, str]], dict[str, dict[str, FitResult]]]:
    calibrated_rows = [dict(row) for row in rows]
    report: dict[str, dict[str, FitResult]] = {}

    for sensor_name, sensor_columns in SENSOR_COLUMN_GROUPS.items():
        report[sensor_name] = {}
        for axis_name, sensor_column in sensor_columns.items():
            raw_column = RAW_COLUMNS[axis_name]
            source = numeric_column(rows, sensor_column)
            target = numeric_column(rows, raw_column)
            fit, corrected = fit_affine(source, target)
            report[sensor_name][axis_name] = fit
            output_column = f"{sensor_column}_calibrated"
            for row, value in zip(calibrated_rows, corrected):
                row[output_column] = f"{value:.10f}"

    return calibrated_rows, report


def write_rows(
    output_path: Path,
    rows: list[dict[str, str]],
    input_fieldnames: list[str],
    dialect: csv.Dialect,
) -> None:
    extra_fields = [
        field
        for field in rows[0].keys()
        if field not in input_fieldnames
    ]
    fieldnames = input_fieldnames + extra_fields
    with output_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, dialect=dialect)
        writer.writeheader()
        writer.writerows(rows)


def write_report(report_path: Path, report: dict[str, dict[str, FitResult]]) -> None:
    payload = {
        sensor_name: {
            axis_name: {
                "scale": axis_result.scale,
                "offset": axis_result.offset,
                "r2": axis_result.r2,
                "rmse": axis_result.rmse,
                "mae": axis_result.mae,
            }
            for axis_name, axis_result in sensor_report.items()
        }
        for sensor_name, sensor_report in report.items()
    }
    report_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def print_report(report: dict[str, dict[str, FitResult]]) -> None:
    for sensor_name, sensor_report in report.items():
        print(sensor_name)
        for axis_name, fit in sensor_report.items():
            print(
                f"  {axis_name}: "
                f"scale={fit.scale:.12f} "
                f"offset={fit.offset:.12f} "
                f"r2={fit.r2:.4f} "
                f"rmse={fit.rmse:.10f} "
                f"mae={fit.mae:.10f}"
            )


def main() -> None:
    args = parse_args()
    input_path = Path(args.input_path)
    output_path = Path(args.output)
    report_path = Path(args.report_json)

    dialect = detect_dialect(input_path)
    rows = read_rows(input_path, dialect)
    if not rows:
        raise SystemExit(f"No rows found in {input_path}")

    calibrated_rows, report = apply_calibration(rows)
    input_fieldnames = list(rows[0].keys())
    write_rows(output_path, calibrated_rows, input_fieldnames, dialect)
    write_report(report_path, report)
    print_report(report)
    print(f"\nWrote calibrated data to {output_path}")
    print(f"Wrote calibration report to {report_path}")


if __name__ == "__main__":
    main()
