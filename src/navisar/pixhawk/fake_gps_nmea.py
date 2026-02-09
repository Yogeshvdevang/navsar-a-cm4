"""Fake GPS Nmea module. Provides fake gps nmea utilities for NAVISAR."""

import datetime as _dt
import math

EARTH_RADIUS_M = 6378137.0


def enu_to_gps(x_e, y_n, z_u, lat0, lon0, alt0):
    """Convert ENU offsets to latitude/longitude/altitude."""
    dlat = y_n / EARTH_RADIUS_M
    dlon = x_e / (EARTH_RADIUS_M * math.cos(math.radians(lat0)))
    lat = lat0 + math.degrees(dlat)
    lon = lon0 + math.degrees(dlon)
    alt = alt0 + z_u
    return lat, lon, alt


def _nmea_checksum(sentence_body):
    """Compute NMEA checksum for a sentence body."""
    checksum = 0
    for ch in sentence_body:
        checksum ^= ord(ch)
    return f"{checksum:02X}"


def _wrap_nmea(sentence_body):
    """Wrap a sentence body with NMEA framing and checksum."""
    return f"${sentence_body}*{_nmea_checksum(sentence_body)}\r\n"


def _format_lat(lat):
    """Format latitude in NMEA degrees/minutes."""
    lat_abs = abs(lat)
    lat_deg = int(lat_abs)
    lat_min = (lat_abs - lat_deg) * 60.0
    lat_dir = "N" if lat >= 0 else "S"
    return f"{lat_deg:02d}{lat_min:07.4f}", lat_dir


def _format_lon(lon):
    """Format longitude in NMEA degrees/minutes."""
    lon_abs = abs(lon)
    lon_deg = int(lon_abs)
    lon_min = (lon_abs - lon_deg) * 60.0
    lon_dir = "E" if lon >= 0 else "W"
    return f"{lon_deg:03d}{lon_min:07.4f}", lon_dir


def _utc_time_fields(now=None):
    """Return NMEA time/date fields in UTC."""
    if now is None:
        now = _dt.datetime.utcnow()
    time_str = now.strftime("%H%M%S")
    frac = f"{now.microsecond / 1_000_000:.2f}"[1:]
    date_str = now.strftime("%d%m%y")
    return f"{time_str}{frac}", date_str


def gga_sentence(
    lat,
    lon,
    alt_m,
    fix_quality=1,
    satellites=10,
    hdop=0.9,
    geoid_sep_m=0.0,
    now=None,
):
    """Build a GGA fix sentence."""
    time_str, _date_str = _utc_time_fields(now)
    lat_str, lat_dir = _format_lat(lat)
    lon_str, lon_dir = _format_lon(lon)
    body = (
        f"GPGGA,{time_str},{lat_str},{lat_dir},"
        f"{lon_str},{lon_dir},{fix_quality},{satellites:02d},"
        f"{hdop:.1f},{alt_m:.1f},M,{geoid_sep_m:.1f},M,,"
    )
    return _wrap_nmea(body)


def rmc_sentence(
    lat,
    lon,
    speed_mps,
    course_deg,
    status="A",
    now=None,
):
    """Build an RMC navigation sentence."""
    time_str, date_str = _utc_time_fields(now)
    lat_str, lat_dir = _format_lat(lat)
    lon_str, lon_dir = _format_lon(lon)
    speed_knots = speed_mps * 1.94384
    body = (
        f"GPRMC,{time_str},{status},{lat_str},{lat_dir},"
        f"{lon_str},{lon_dir},{speed_knots:.1f},{course_deg:.1f},{date_str},,,A"
    )
    return _wrap_nmea(body)


def speed_course_from_enu(vx_e, vy_n):
    """Compute ground speed and course from ENU velocities."""
    speed = math.hypot(vx_e, vy_n)
    if speed < 1e-3:
        return 0.0, 0.0
    course_rad = math.atan2(vx_e, vy_n)
    course_deg = (math.degrees(course_rad) + 360.0) % 360.0
    return speed, course_deg
