"""Mode for sending GPS over a serial port."""

from navisar.modes.common import EnuVelocityTracker, enu_to_gps
from navisar.pixhawk.fake_gps_nmea import speed_course_from_enu


def _bytes_hex(payload):
    return " ".join(f"{b:02X}" for b in payload)


def _finite(value):
    return value is not None and float(value) == float(value) and abs(float(value)) != float("inf")


def _clamp(value, low, high):
    return max(low, min(high, value))


MAX_GPS_SPEED_MPS = 100.0


class GpsPortMode:
    """Send NMEA/UBX GPS output derived from ENU camera drift."""
    def __init__(
        self,
        emitter,
        nmea_emitter,
        ubx_emitter,
        print_enabled,
        warn_interval_s=2.0,
    ):
        self.emitter = emitter
        self.nmea_emitter = nmea_emitter
        self.ubx_emitter = ubx_emitter
        self.print_enabled = bool(print_enabled)
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
        alt_override_m=None,
        heading_deg=None,
        heading_only=False,
    ):
        """Send GPS sentences over serial if ready."""
        if origin is None:
            self._warn(now, "GPS->PORT: missing gps_origin in pixhawk.yaml.")
            return
        if self.nmea_emitter is None and self.ubx_emitter is None:
            self._warn(now, "GPS->PORT: gps_output disabled; nothing to send.")
            return
        if not self.emitter.ready(now):
            return

        lat, lon, _alt_m = enu_to_gps(x_m, y_m, z_m, origin)
        if alt_override_m is None:
            self._warn(now, "GPS->PORT: barometer altitude unavailable; skipping send.")
            return
        alt_base = 0.0 if origin[2] is None else float(origin[2])
        alt_m = alt_base + float(alt_override_m)
        if abs(alt_m) < 1e-3:
            alt_m = 0.0
        if not (_finite(lat) and _finite(lon) and _finite(alt_m)):
            self._warn(now, "GPS->PORT: invalid lat/lon/alt; skipping send.")
            return
        lat = _clamp(lat, -90.0, 90.0)
        lon = _clamp(lon, -180.0, 180.0)

        vx_enu, vy_enu, vz_enu = self._vel_tracker.velocity_and_update(
            now, x_m, y_m, z_m
        )
        if not (_finite(vx_enu) and _finite(vy_enu) and _finite(vz_enu)):
            vx_enu, vy_enu, vz_enu = 0.0, 0.0, 0.0
        vx_enu = _clamp(vx_enu, -MAX_GPS_SPEED_MPS, MAX_GPS_SPEED_MPS)
        vy_enu = _clamp(vy_enu, -MAX_GPS_SPEED_MPS, MAX_GPS_SPEED_MPS)
        vz_enu = _clamp(vz_enu, -MAX_GPS_SPEED_MPS, MAX_GPS_SPEED_MPS)

        heading_override = float(heading_deg) if _finite(heading_deg) else None
        if heading_only and heading_override is None:
            self._warn(now, "GPS->PORT: compass heading unavailable; forcing 0°.")
            heading_override = 0.0
        if heading_only and heading_override is not None:
            self._warn(
                now, f"GPS->PORT: using compass heading {heading_override:.1f}°."
            )
        nmea_payload = None
        ubx_payload = None
        if self.nmea_emitter is not None and self.nmea_emitter.ready(now):
            course_override = heading_override if heading_only or heading_override is not None else None
            nmea_payload = self.nmea_emitter.send(
                lat,
                lon,
                alt_m,
                vx_enu,
                vy_enu,
                course_deg_override=course_override,
                force_heading=heading_only,
            )
        if self.ubx_emitter is not None and self.ubx_emitter.ready(now):
            course_override = heading_override if heading_only or heading_override is not None else None
            ubx_payload = self.ubx_emitter.send(
                lat,
                lon,
                alt_m,
                vx_enu,
                vy_enu,
                course_deg_override=course_override,
                force_heading=heading_only,
            )
        if nmea_payload or ubx_payload:
            speed_mps, computed_heading_deg = speed_course_from_enu(vx_enu, vy_enu)
            heading_deg = heading_override if heading_override is not None else computed_heading_deg
            self.last_payload = {
                "time_s": now,
                "lat": lat,
                "lon": lon,
                "alt_m": alt_m,
                "vx_enu": vx_enu,
                "vy_enu": vy_enu,
                "speed_mps": speed_mps,
                "heading_deg": heading_deg,
                "nmea": {
                    "gga_hex": _bytes_hex(nmea_payload["gga"]) if nmea_payload else None,
                    "rmc_hex": _bytes_hex(nmea_payload["rmc"]) if nmea_payload else None,
                    "sats": nmea_payload.get("sats") if nmea_payload else None,
                },
                "ubx": {
                    "pvt_hex": _bytes_hex(ubx_payload["pvt"]) if ubx_payload else None,
                    "posllh_hex": _bytes_hex(ubx_payload["posllh"]) if ubx_payload else None,
                    "velned_hex": _bytes_hex(ubx_payload["velned"]) if ubx_payload else None,
                    "sol_hex": _bytes_hex(ubx_payload["sol"]) if ubx_payload else None,
                    "sats": ubx_payload.get("sats") if ubx_payload else None,
                },
            }

        if self.print_enabled and (nmea_payload or ubx_payload):
            speed_mps, computed_heading_deg = speed_course_from_enu(vx_enu, vy_enu)
            heading_deg = heading_override if heading_override is not None else computed_heading_deg
            sats_display = None
            if nmea_payload and "sats" in nmea_payload:
                sats_display = nmea_payload["sats"]
            elif ubx_payload and "sats" in ubx_payload:
                sats_display = ubx_payload["sats"]
            if nmea_payload and ubx_payload:
                sent_label = "UBX-NAV-PVT, POSLLH, VELNED, SOL + NMEA GGA, RMC"
                prefix = "GPS->PORT"
            elif ubx_payload:
                sent_label = "UBX-NAV-PVT, POSLLH, VELNED, SOL"
                prefix = "GPS->UBX"
            else:
                sent_label = "NMEA GGA, RMC"
                prefix = "GPS->NMEA"
            sats_text = "n/a" if sats_display is None else str(sats_display)
            print(
                f"{prefix}:\n"
                f"LAT : {abs(lat):.7f}° {'N' if lat >= 0 else 'S'}\n"
                f"LON : {abs(lon):.7f}° {'E' if lon >= 0 else 'W'}\n"
                f"ALT : {alt_m:.1f} m\n"
                f"SPD : {speed_mps:.2f} m/s\n"
                f"HDG : {heading_deg:.1f}°\n"
                f"SATS: {sats_text}\n"
                f"Sent: {sent_label}"
            )
            print("-" * 50)

        self.emitter.mark_sent(now)
