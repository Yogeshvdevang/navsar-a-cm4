"""Xy Drift module. Provides xy drift utilities for NAVISAR."""

import time

from navisar.main import build_vo_pipeline


def main():
    """Run VO and print XY drift telemetry."""
    vo, _mavlink_interface = build_vo_pipeline()
    last_print = 0.0
    print_interval_s = 0.2

    def on_update(x, y, z, dx_m, dy_m, dz_m, dx_pixels, dy_pixels, inliers):
        nonlocal last_print
        now = time.time()
        if now - last_print < print_interval_s:
            return
        last_print = now
        print(
            f"X={x:.3f} Y={y:.3f} Z={z:.3f} "
            f"dX={dx_m:.3f} dY={dy_m:.3f} dZ={dz_m:.3f} "
            f"dx_pix={dx_pixels:.2f} dy_pix={dy_pixels:.2f} inliers={inliers}"
        )

    vo.run(window_name="Camera Drift XY", on_update=on_update)


if __name__ == "__main__":
    main()
