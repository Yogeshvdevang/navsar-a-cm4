from types import SimpleNamespace

from navisar.modes.optical_flow_gps_port import OpticalFlowGpsPortMode


class _FakeGpsPortMode:
    def __init__(self):
        self.last_payload = None

    def handle(
        self,
        now,
        x_m,
        y_m,
        z_m,
        origin,
        alt_override_m=None,
        nav_pvt_alt_mm_override=None,
        heading_deg=None,
        send_heading=True,
        heading_only=False,
        apply_final_altitude_offset=False,
    ):
        self.last_payload = {
            "time_s": now,
            "lat": origin[0],
            "lon": origin[1],
            "alt_m": alt_override_m,
            "vx_enu": 0.0,
            "vy_enu": 0.0,
            "speed_mps": 0.0,
            "heading_deg": heading_deg,
            "ubx": {"pvt_hex": "01"},
        }


def _sample(
    *,
    time_ms,
    distance_mm=1000,
    flow_vx=0,
    flow_vy=0,
    flow_quality=100,
    flow_ok=1,
    dist_ok=1,
    speed_x=1.0,
    speed_y=0.0,
):
    return SimpleNamespace(
        time_ms=time_ms,
        distance_mm=distance_mm,
        flow_vx=flow_vx,
        flow_vy=flow_vy,
        flow_quality=flow_quality,
        flow_ok=flow_ok,
        dist_ok=dist_ok,
        speed_x=speed_x,
        speed_y=speed_y,
    )


def test_invalid_sample_decays_velocity_instead_of_zeroing():
    gps_port_mode = _FakeGpsPortMode()
    mode = OpticalFlowGpsPortMode(
        gps_port_mode=gps_port_mode,
        min_quality=30,
        smoothing_alpha=0.3,
        deadband_mps=0.001,
        print_enabled=False,
    )

    origin = (12.0, 77.0, None)
    mode.handle(1.0, _sample(time_ms=1000), origin, alt_override_m=1.0, heading_deg=90.0)
    mode.handle(1.1, _sample(time_ms=1100), origin, alt_override_m=1.0, heading_deg=90.0)
    mode.handle(
        1.2,
        _sample(time_ms=1200, flow_ok=0, dist_ok=0, speed_x=0.0, speed_y=0.0),
        origin,
        alt_override_m=1.0,
        heading_deg=90.0,
    )

    payload = mode.last_payload
    assert payload is not None
    assert payload["optical_flow"]["speed_x_mps_used"] > 0.0
    assert payload["optical_flow"]["x_m"] > 0.0

