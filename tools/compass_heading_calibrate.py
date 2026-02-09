#!/usr/bin/env python3
"""Interactive heading display + yaw offset calibration.

python3 tools/compass_heading_calibrate.py

How to use:
- Place a real compass pointing North.
- Align the drone so its nose points North too.
- Run this script and press 'm' to compute heading_offset_deg.
- Paste the output into config/pixhawk.yaml under compass.calibration.
"""

import os
import sys
import time
import termios
import tty

ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SRC_DIR = os.path.join(ROOT_DIR, "src")
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

import yaml

from navisar.sensors.compass import CompassReader


def _dir_from_heading(deg):
    dirs = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
    idx = int((deg + 22.5) // 45) % 8
    return dirs[idx]


def _read_config_calibration():
    cfg_path = os.path.join(ROOT_DIR, "config", "pixhawk.yaml")
    if not os.path.exists(cfg_path):
        return {}, None
    with open(cfg_path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}
    compass_cfg = cfg.get("compass", {}) if isinstance(cfg, dict) else {}
    return compass_cfg.get("calibration", {}) or {}, cfg_path


def main():
    calibration, cfg_path = _read_config_calibration()
    if calibration:
        print("Loaded compass.calibration from config/pixhawk.yaml")
        if "heading_offset_deg" in calibration:
            print("Note: existing heading_offset_deg will be overridden by this result")
    else:
        print("No compass.calibration found in config; using raw compass data")

    reader = CompassReader(preferred_bus=1, calibration=calibration)
    print(f"Compass: bus {reader.bus_index}, addr=0x{reader.addr:02X}")
    print("Press 'm' to capture North alignment. Press 'q' to quit.")

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while True:
            heading_deg, _ = reader.read_milligauss()
            heading_deg = float(heading_deg) % 360.0
            direction = _dir_from_heading(heading_deg)
            print(f"Heading: {direction} {heading_deg:6.2f}°", end="\r", flush=True)

            ch = None
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                ch = sys.stdin.read(1)
            if ch == "q":
                print("\nExiting.")
                break
            if ch == "m":
                offset = (-heading_deg) % 360.0
                print("\nCaptured North alignment.")
                print("Add this to config/pixhawk.yaml under compass.calibration:")
                print(f"  heading_offset_deg: {offset:.2f}")
                if cfg_path:
                    print(f"Config path: {cfg_path}")
                break
            time.sleep(0.05)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        reader.close()


if __name__ == "__main__":
    import select
    main()
