"""Mode for sending GPS_INPUT via MAVLink."""

from navisar.modes.common import EnuVelocityTracker, enu_to_gps
from navisar.pixhawk.fake_gps_nmea import speed_course_from_enu


def _bytes_hex(payload):
    return " ".join(f"{b:02X}" for b in payload)


def _finite(value):
    return value is not None and float(value) == float(value) and abs(float(value)) != float("inf")


def _clamp(value, low, high):
    return max(low, min(high, value))


MAX_GPS_SPEED_MPS = 100.0
MAX_YAW_CDEG = 35999


class GpsMavlinkMode:
    """Send MAVLink GPS_INPUT from ENU camera drift."""
    def __init__(
        self,
        emitter,
        fix_type,
        satellites,
        print_enabled,
        ignore_flags=0,
        warn_interval_s=2.0,
    ):
        self.emitter = emitter
        self.fix_type = int(fix_type)
        self.satellites = int(satellites)
        self.print_enabled = bool(print_enabled)
        self.ignore_flags = int(ignore_flags)
        self.warn_interval_s = float(warn_interval_s)
        self._last_warn = 0.0
        self._vel_tracker = EnuVelocityTracker()
        self.last_payload = None

    def _warn(self, now, message):
        if now - self._last_warn >= self.warn_interval_s:
            print(message)
            self._last_warn = now

    def handle(
        self,
        now,
        x_m,
        y_m,
        z_m,
        origin,
        mavlink_interface,
        alt_override_m=None,
        vz_override_mps=None,
        speed_accuracy_mps=None,
        gps_fix=None,
        yaw_deg=None,
    ):
        """Send GPS_INPUT if ready."""
        if mavlink_interface is None:
            self._warn(now, "GPS->MAV: MAVLink unavailable; cannot send GPS_INPUT.")
            return
        if origin is None:
            self._warn(now, "GPS->MAV: missing gps_origin in pixhawk.yaml.")
            return
        if not self.emitter.ready(now):
            return

        lat, lon, alt_m = enu_to_gps(x_m, y_m, z_m, origin)
        barometer_altitude_m = None
        if alt_override_m is not None:
            alt_m = alt_override_m
            barometer_altitude_m = float(alt_override_m)
        use_barometer_altitude = barometer_altitude_m is not None
        fix_type = self.fix_type
        satellites = self.satellites
        if gps_fix:
            fix_lat = gps_fix.get("lat")
            fix_lon = gps_fix.get("lon")
            fix_alt = gps_fix.get("alt_m")
            if _finite(fix_lat) and _finite(fix_lon):
                lat = float(fix_lat)
                lon = float(fix_lon)
            if (not use_barometer_altitude) and _finite(fix_alt):
                alt_m = float(fix_alt)
            fix_type = int(gps_fix.get("fix_type", fix_type))
            sats = gps_fix.get("sats")
            if sats is not None:
                satellites = int(sats)
        if not (_finite(lat) and _finite(lon) and _finite(alt_m)):
            self._warn(now, "GPS->MAV: invalid lat/lon/alt; skipping send.")
            return
        lat = _clamp(lat, -90.0, 90.0)
        lon = _clamp(lon, -180.0, 180.0)

        yaw_cdeg = None
        if _finite(yaw_deg):
            yaw_norm = float(yaw_deg) % 360.0
            yaw_cdeg = int(round(yaw_norm * 100.0))
            yaw_cdeg = _clamp(yaw_cdeg, 0, MAX_YAW_CDEG)

        vx_enu, vy_enu, vz_enu = self._vel_tracker.velocity_and_update(now, x_m, y_m, z_m)
        if vz_override_mps is not None:
            vz_enu = float(vz_override_mps)
        if not (_finite(vx_enu) and _finite(vy_enu) and _finite(vz_enu)):
            vx_enu, vy_enu, vz_enu = 0.0, 0.0, 0.0
        vx_enu = _clamp(vx_enu, -MAX_GPS_SPEED_MPS, MAX_GPS_SPEED_MPS)
        vy_enu = _clamp(vy_enu, -MAX_GPS_SPEED_MPS, MAX_GPS_SPEED_MPS)
        vz_enu = _clamp(vz_enu, -MAX_GPS_SPEED_MPS, MAX_GPS_SPEED_MPS)

        raw_payload = mavlink_interface.send_gps_input(
            lat,
            lon,
            alt_m,
            fix_type=fix_type,
            satellites_visible=satellites,
            vn=vy_enu,
            ve=vx_enu,
            vd=-vz_enu,
            speed_accuracy=0.5 if speed_accuracy_mps is None else speed_accuracy_mps,
            yaw_cdeg=yaw_cdeg,
            ignore_flags=self.ignore_flags,
        )
        speed_mps, heading_deg = speed_course_from_enu(vx_enu, vy_enu)
        payload_for_dashboard = None
        if raw_payload is not None:
            payload_for_dashboard = dict(raw_payload)
            raw_bytes = payload_for_dashboard.pop("raw", None)
            if raw_bytes is not None:
                payload_for_dashboard["raw_hex"] = _bytes_hex(raw_bytes)
        hdop = payload_for_dashboard.get("hdop") if payload_for_dashboard else None
        vdop = payload_for_dashboard.get("vdop") if payload_for_dashboard else None
        horiz_accuracy = (
            payload_for_dashboard.get("horiz_accuracy")
            if payload_for_dashboard
            else None
        )
        vert_accuracy = (
            payload_for_dashboard.get("vert_accuracy")
            if payload_for_dashboard
            else None
        )
        speed_accuracy = (
            payload_for_dashboard.get("speed_accuracy")
            if payload_for_dashboard
            else (0.5 if speed_accuracy_mps is None else speed_accuracy_mps)
        )
        satellites_visible = (
            payload_for_dashboard.get("satellites_visible")
            if payload_for_dashboard
            else satellites
        )
        self.last_payload = {
            "time_s": now,
            "lat": lat,
            "lon": lon,
            "alt_m": alt_m,
            "barometer_altitude_m": barometer_altitude_m,
            "altitude_source": "barometer" if use_barometer_altitude else "gps_or_fused",
            "vn": vy_enu,
            "ve": vx_enu,
            "vd": -vz_enu,
            "fix_type": fix_type,
            "satellites": satellites,
            "speed_mps": speed_mps,
            "heading_deg": heading_deg,
            "course_deg": heading_deg,
            "vel_n_mps": vy_enu,
            "vel_e_mps": vx_enu,
            "vel_d_mps": -vz_enu,
            "raw_hex": _bytes_hex(raw_payload["raw"]) if raw_payload else None,
            "speed_accuracy_mps": speed_accuracy,
            "speed_accuracy": speed_accuracy,
            "hdop": hdop,
            "vdop": vdop,
            "pdop": None,
            "horizontal_accuracy_m": horiz_accuracy,
            "vertical_accuracy_m": vert_accuracy,
            "satellites_visible": satellites_visible,
            "mavlink_time_usec": payload_for_dashboard.get("time_usec") if payload_for_dashboard else None,
            "mavlink_time_week": payload_for_dashboard.get("time_week") if payload_for_dashboard else None,
            "mavlink_time_week_ms": payload_for_dashboard.get("time_week_ms") if payload_for_dashboard else None,
            "message_hex": _bytes_hex(raw_payload["raw"]) if raw_payload else None,
            "yaw_cdeg": yaw_cdeg,
            "yaw_deg": yaw_deg,
            "payload": payload_for_dashboard,
        }

        if self.print_enabled:
            if raw_payload and raw_payload.get("raw"):
                print(f"GPS_INPUT RAW: {_bytes_hex(raw_payload['raw'])}")
                print("GPS_INPUT FIELDS:")
                print(
                    f"time_usec: {raw_payload['time_usec']}  "
                    f"time_week_ms: {raw_payload['time_week_ms']}  "
                    f"time_week: {raw_payload['time_week']}"
                )
                print(
                    f"gps_id: {raw_payload['gps_id']}  "
                    f"ignore_flags: {raw_payload['ignore_flags']}  "
                    f"fix_type: {raw_payload['fix_type']}"
                )
                print(
                    f"lat: {raw_payload['lat']:.7f}  "
                    f"lon: {raw_payload['lon']:.7f}  "
                    f"alt_m: {raw_payload['alt_m']:.2f}"
                )
                print(
                    f"hdop: {raw_payload['hdop']:.2f}  "
                    f"vdop: {raw_payload['vdop']:.2f}  "
                    f"vn: {raw_payload['vn']:.3f}  "
                    f"ve: {raw_payload['ve']:.3f}  "
                    f"vd: {raw_payload['vd']:.3f}"
                )
                print(
                    f"speed_accuracy: {raw_payload['speed_accuracy']:.2f}  "
                    f"horiz_accuracy: {raw_payload['horiz_accuracy']:.2f}  "
                    f"vert_accuracy: {raw_payload['vert_accuracy']:.2f}  "
                    f"satellites_visible: {raw_payload['satellites_visible']}"
                )
                print("-" * 50)
            else:
                print("GPS_INPUT: no payload available to print.")

        self.emitter.mark_sent(now)
