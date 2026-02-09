"""GPS Serial module. Provides gps serial utilities for NAVISAR."""

import time

import serial
from serial.tools import list_ports

DEFAULT_BAUDS = [4800, 9600, 19200, 38400, 57600, 115200]
DEFAULT_PROBE_SECONDS = 3.0


def _detect_ports():
    ports = list(list_ports.comports())
    if not ports:
        return []
    preferred = []
    fallback = []
    for port in ports:
        desc = (port.description or "").lower()
        device = port.device or ""
        if "/dev/serial/by-id" in device:
            preferred.append(device)
        elif device.startswith(("/dev/ttyUSB", "/dev/ttyACM", "/dev/ttyAMA", "/dev/ttyS")):
            preferred.append(device)
        elif "usb" in desc or "uart" in desc or "cp210" in desc or "ch340" in desc:
            preferred.append(device)
        else:
            fallback.append(device)
    return preferred + fallback


def _read_nmea(ser, seconds, verbose=True):
    end_time = time.time() + seconds
    while time.time() < end_time:
        line = ser.readline()
        if not line:
            continue
        if line.startswith(b"$"):
            if verbose:
                try:
                    print(line.decode("ascii", errors="ignore").strip())
                except Exception:
                    pass
            return True
    return False


def find_gps_port_and_baud(
    port=None,
    bauds=None,
    probe_seconds=DEFAULT_PROBE_SECONDS,
    verbose=True,
):
    ports = [port] if port else _detect_ports()
    if not ports:
        return None
    for candidate in ports:
        for baud in bauds or DEFAULT_BAUDS:
            try:
                if verbose:
                    print(f"Probing on {candidate} @ {baud} baud for {probe_seconds}s...")
                with serial.Serial(candidate, baud, timeout=1) as ser:
                    if _read_nmea(ser, probe_seconds, verbose=verbose):
                        return candidate, baud
            except serial.SerialException as exc:
                if verbose:
                    print(f"Failed to open {candidate} @ {baud}: {exc}")
    return None


def probe_nmea_on_port(port, baud=9600, seconds=3.0, verbose=True):
    """Return True if NMEA data is seen on the port within the time window."""
    if not port:
        return False
    try:
        with serial.Serial(port, baud, timeout=1) as ser:
            return _read_nmea(ser, seconds, verbose=verbose)
    except serial.SerialException as exc:
        if verbose:
            print(f"GPS probe failed on {port} @ {baud}: {exc}")
    return False


class GpsSerialReader:
    """Read NMEA GPS data from a serial port."""
    def __init__(
        self,
        port,
        baud=9600,
        fmt="auto",
        probe_seconds=DEFAULT_PROBE_SECONDS,
        bauds=None,
        verbose=True,
    ):
        """Open the serial port and set parse format."""
        port_is_auto = port is None or str(port).lower() == "auto"
        baud_is_auto = baud is None or str(baud).lower() == "auto"
        if port_is_auto or baud_is_auto:
            choice = find_gps_port_and_baud(
                port=None if port_is_auto else port,
                bauds=bauds,
                probe_seconds=probe_seconds,
                verbose=verbose,
            )
            if not choice:
                raise RuntimeError("No NMEA data received. Check wiring, port, and baud rate.")
            port, baud = choice
            if verbose:
                print(f"Locked GPS on {port} @ {baud}")
        self.port = port
        self.baud = baud
        self.fmt = (fmt or "auto").lower()
        self._ser = serial.Serial(port, baud, timeout=0)
        self._last_fix = None
        self._last_time = None

    def read_messages(self, max_lines=10):
        """Read up to max_lines and return latest fix/time."""
        for _ in range(max_lines):
            line = self._ser.readline()
            if not line:
                break
            fix = self._parse_line(line)
            if fix is not None:
                self._last_fix = fix
                self._last_time = time.time()
        return self._last_fix, self._last_time

    def _parse_line(self, raw):
        """Decode a raw line and parse supported NMEA messages."""
        if not raw:
            return None
        if self.fmt in ("auto", "nmea"):
            if raw.startswith(b"$"):
                try:
                    text = raw.decode("ascii", errors="ignore").strip()
                except Exception:
                    return None
                return _parse_nmea(text)
        return None


def parse_nmea_sentence(sentence):
    """Parse a single NMEA sentence into a fix dict."""
    if not sentence:
        return None
    if isinstance(sentence, bytes):
        try:
            sentence = sentence.decode("ascii", errors="ignore")
        except Exception:
            return None
    return _parse_nmea(str(sentence).strip())


def parse_nmea_stream(lines):
    """Return the latest valid fix from an iterable of NMEA sentences."""
    latest = None
    for line in lines:
        fix = parse_nmea_sentence(line)
        if fix is not None:
            latest = fix
    return latest


def _parse_nmea(line):
    """Parse an NMEA sentence and return a fix dict."""
    if not line.startswith("$"):
        return None
    if "*" in line:
        line = line.split("*", 1)[0]
    fields = line.split(",")
    if not fields:
        return None
    msg = fields[0][3:] if len(fields[0]) >= 6 else fields[0]
    if msg.endswith("GGA"):
        return _parse_gga(fields)
    if msg.endswith("RMC"):
        return _parse_rmc(fields)
    return None


def _parse_gga(fields):
    """Parse a GGA sentence into a fix dict."""
    if len(fields) < 10:
        return None
    lat = _nmea_to_decimal(fields[2], fields[3])
    lon = _nmea_to_decimal(fields[4], fields[5])
    fix_quality = _safe_int(fields[6])
    sats = _safe_int(fields[7])
    alt = _safe_float(fields[9])
    if lat is None or lon is None:
        return None
    fix_type = 3 if fix_quality and fix_quality > 0 else 0
    return {
        "lat": lat,
        "lon": lon,
        "alt_m": alt,
        "fix_type": fix_type,
        "sats": sats,
    }


def _parse_rmc(fields):
    """Parse an RMC sentence into a fix dict."""
    if len(fields) < 7:
        return None
    status = fields[2] if len(fields) > 2 else ""
    lat = _nmea_to_decimal(fields[3], fields[4])
    lon = _nmea_to_decimal(fields[5], fields[6])
    if lat is None or lon is None:
        return None
    fix_type = 3 if status == "A" else 0
    return {
        "lat": lat,
        "lon": lon,
        "alt_m": None,
        "fix_type": fix_type,
        "sats": None,
    }


def _nmea_to_decimal(value, hemi):
    """Convert NMEA lat/lon fields into decimal degrees."""
    if not value or not hemi:
        return None
    try:
        val = float(value)
    except ValueError:
        return None
    hemi = hemi.upper()
    if hemi in ("N", "S"):
        deg = int(val / 100)
        minutes = val - (deg * 100)
        dec = deg + minutes / 60.0
        if hemi == "S":
            dec = -dec
        return dec
    if hemi in ("E", "W"):
        deg = int(val / 100)
        minutes = val - (deg * 100)
        dec = deg + minutes / 60.0
        if hemi == "W":
            dec = -dec
        return dec
    return None


def _safe_int(value):
    """Convert to int, returning None on failure."""
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _safe_float(value):
    """Convert to float, returning None on failure."""
    try:
        return float(value)
    except (TypeError, ValueError):
        return None
