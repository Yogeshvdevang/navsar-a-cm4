#!/usr/bin/env python3
"""
Compare GPS and optical-flow position values from a CSV log.
"""

import argparse
import csv
import math
from pathlib import Path
from xml.sax.saxutils import escape


DEFAULT_CSV = Path(__file__).resolve().parent / "logs" / "gps_flow_log_20260317_145034.csv"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Compare GPS and optical-flow latitude, longitude, and altitude from a CSV log."
    )
    parser.add_argument(
        "csv_file",
        nargs="?",
        default=str(DEFAULT_CSV),
        help=f"Path to CSV log file (default: {DEFAULT_CSV})",
    )
    parser.add_argument(
        "--limit",
        type=int,
        default=10,
        help="Number of sample rows to print from the comparison table (default: 10)",
    )
    parser.add_argument(
        "--svg",
        default="sim_csv_plot.svg",
        help="Output SVG graph path (default: sim_csv_plot.svg)",
    )
    return parser.parse_args()


def safe_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def mean(values):
    return sum(values) / len(values) if values else math.nan


def rms(values):
    return math.sqrt(sum(v * v for v in values) / len(values)) if values else math.nan


def fmt(value, digits=7):
    if math.isnan(value):
        return "nan"
    return f"{value:.{digits}f}"


def load_rows(csv_path):
    rows = []
    with csv_path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            gps_lat = safe_float(row.get("gps_latitude_deg"))
            gps_lon = safe_float(row.get("gps_longitude_deg"))
            gps_alt = safe_float(row.get("gps_altitude_m"))
            of_lat = safe_float(row.get("of_latitude_deg"))
            of_lon = safe_float(row.get("of_longitude_deg"))
            of_alt = safe_float(row.get("of_altitude_m"))

            values = [gps_lat, gps_lon, gps_alt, of_lat, of_lon, of_alt]
            if any(math.isnan(v) for v in values):
                continue

            rows.append(
                {
                    "timestamp": row.get("timestamp_utc", ""),
                    "gps_lat": gps_lat,
                    "gps_lon": gps_lon,
                    "gps_alt": gps_alt,
                    "of_lat": of_lat,
                    "of_lon": of_lon,
                    "of_alt": of_alt,
                    "lat_diff": of_lat - gps_lat,
                    "lon_diff": of_lon - gps_lon,
                    "alt_diff": of_alt - gps_alt,
                }
            )
    return rows


def print_summary(rows):
    lat_diffs = [row["lat_diff"] for row in rows]
    lon_diffs = [row["lon_diff"] for row in rows]
    alt_diffs = [row["alt_diff"] for row in rows]

    print(f"Rows compared: {len(rows)}")
    print()
    print("Difference summary (optical_flow - gps)")
    print(f"Latitude  mean: {fmt(mean(lat_diffs), 9)} deg")
    print(f"Latitude   rms: {fmt(rms(lat_diffs), 9)} deg")
    print(f"Latitude   min: {fmt(min(lat_diffs), 9)} deg")
    print(f"Latitude   max: {fmt(max(lat_diffs), 9)} deg")
    print()
    print(f"Longitude mean: {fmt(mean(lon_diffs), 9)} deg")
    print(f"Longitude  rms: {fmt(rms(lon_diffs), 9)} deg")
    print(f"Longitude  min: {fmt(min(lon_diffs), 9)} deg")
    print(f"Longitude  max: {fmt(max(lon_diffs), 9)} deg")
    print()
    print(f"Altitude  mean: {fmt(mean(alt_diffs), 3)} m")
    print(f"Altitude   rms: {fmt(rms(alt_diffs), 3)} m")
    print(f"Altitude   min: {fmt(min(alt_diffs), 3)} m")
    print(f"Altitude   max: {fmt(max(alt_diffs), 3)} m")


def print_samples(rows, limit):
    print()
    print("Sample comparison rows")
    print(
        "timestamp_utc                    "
        "gps_lat      of_lat       d_lat        "
        "gps_lon      of_lon       d_lon        "
        "gps_alt   of_alt    d_alt"
    )
    for row in rows[:limit]:
        print(
            f"{row['timestamp']:<32} "
            f"{row['gps_lat']:.7f}  {row['of_lat']:.7f}  {row['lat_diff']:+.9f}  "
            f"{row['gps_lon']:.7f}  {row['of_lon']:.7f}  {row['lon_diff']:+.9f}  "
            f"{row['gps_alt']:.3f}  {row['of_alt']:.3f}  {row['alt_diff']:+.3f}"
        )


def value_to_y(value, min_value, max_value, top, height):
    if math.isclose(max_value, min_value):
        return top + height / 2
    scale = (value - min_value) / (max_value - min_value)
    return top + height - scale * height


def build_polyline(values, min_value, max_value, left, top, width, height):
    if len(values) == 1:
        x = left + width / 2
        y = value_to_y(values[0], min_value, max_value, top, height)
        return f"{x:.2f},{y:.2f}"

    points = []
    for index, value in enumerate(values):
        x = left + (index / (len(values) - 1)) * width
        y = value_to_y(value, min_value, max_value, top, height)
        points.append(f"{x:.2f},{y:.2f}")
    return " ".join(points)


def render_panel(parts, title, gps_values, of_values, diffs, top, width, height, left, right):
    plot_width = width - left - right
    plot_height = height - 70
    plot_top = top + 35

    all_values = gps_values + of_values
    min_value = min(all_values)
    max_value = max(all_values)
    pad = (max_value - min_value) * 0.08 if not math.isclose(max_value, min_value) else 1.0
    min_value -= pad
    max_value += pad

    gps_line = build_polyline(gps_values, min_value, max_value, left, plot_top, plot_width, plot_height)
    of_line = build_polyline(of_values, min_value, max_value, left, plot_top, plot_width, plot_height)

    parts.append(f'<text x="{left}" y="{top + 18}" font-size="16" font-weight="bold" fill="#1f2937">{escape(title)}</text>')
    parts.append(
        f'<text x="{left}" y="{top + 285}" font-size="12" fill="#4b5563">'
        f'range {fmt(min(all_values), 4)} to {fmt(max(all_values), 4)} | '
        f'diff mean {fmt(mean(diffs), 6)} | diff rms {fmt(rms(diffs), 6)}</text>'
    )
    parts.append(
        f'<rect x="{left}" y="{plot_top}" width="{plot_width}" height="{plot_height}" '
        'fill="#ffffff" stroke="#d1d5db" stroke-width="1"/>'
    )

    for step in range(5):
        y = plot_top + step * (plot_height / 4)
        parts.append(
            f'<line x1="{left}" y1="{y:.2f}" x2="{left + plot_width}" y2="{y:.2f}" '
            'stroke="#e5e7eb" stroke-width="1"/>'
        )

    parts.append(
        f'<polyline fill="none" stroke="#2563eb" stroke-width="2" points="{gps_line}"/>'
    )
    parts.append(
        f'<polyline fill="none" stroke="#dc2626" stroke-width="2" points="{of_line}"/>'
    )


def write_svg(rows, svg_path, csv_path):
    width = 1400
    height = 980
    left = 80
    right = 40
    panel_height = 290
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#f8fafc"/>',
        '<text x="80" y="40" font-size="24" font-weight="bold" fill="#111827">GPS vs Optical Flow Comparison</text>',
        f'<text x="80" y="65" font-size="13" fill="#475569">{escape(str(csv_path))}</text>',
        '<text x="80" y="88" font-size="13" fill="#2563eb">Blue: GPS</text>',
        '<text x="170" y="88" font-size="13" fill="#dc2626">Red: Optical Flow</text>',
        '<text x="320" y="88" font-size="13" fill="#475569">X-axis: sample index</text>',
    ]

    render_panel(
        parts,
        "Latitude (deg)",
        [row["gps_lat"] for row in rows],
        [row["of_lat"] for row in rows],
        [row["lat_diff"] for row in rows],
        110,
        width,
        panel_height,
        left,
        right,
    )
    render_panel(
        parts,
        "Longitude (deg)",
        [row["gps_lon"] for row in rows],
        [row["of_lon"] for row in rows],
        [row["lon_diff"] for row in rows],
        400,
        width,
        panel_height,
        left,
        right,
    )
    render_panel(
        parts,
        "Altitude (m)",
        [row["gps_alt"] for row in rows],
        [row["of_alt"] for row in rows],
        [row["alt_diff"] for row in rows],
        690,
        width,
        panel_height,
        left,
        right,
    )

    parts.append("</svg>")
    svg_path.write_text("\n".join(parts), encoding="utf-8")


def main():
    args = parse_args()
    csv_path = Path(args.csv_file).expanduser().resolve()
    svg_path = Path(args.svg).expanduser().resolve()

    if not csv_path.is_file():
        raise SystemExit(f"CSV file not found: {csv_path}")

    rows = load_rows(csv_path)
    if not rows:
        raise SystemExit("No valid rows found with both GPS and optical-flow lat/lon/alt values.")

    print(f"CSV file: {csv_path}")
    print_summary(rows)
    print_samples(rows, max(args.limit, 0))
    write_svg(rows, svg_path, csv_path)
    print()
    print(f"SVG graph saved to: {svg_path}")


if __name__ == "__main__":
    main()
