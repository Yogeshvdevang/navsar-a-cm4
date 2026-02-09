"""Compass (HMC5883L/QMC5883L) utilities for NAVISAR."""

import glob
import math

try:
    from smbus2 import SMBus
except ImportError:  # pragma: no cover - depends on platform packages
    from smbus import SMBus  # type: ignore

HMC5883L_ADDR = 0x1E
QMC5883L_ADDR = 0x0D


def list_i2c_bus_indices(preferred=None):
    """Return available I2C bus indices, preferring the requested one."""
    paths = sorted(glob.glob("/dev/i2c-*"))
    indices = []
    for path in paths:
        try:
            indices.append(int(path.split("-")[-1]))
        except ValueError:
            continue
    if preferred is not None and preferred in indices:
        indices.remove(preferred)
        indices.insert(0, preferred)
    return indices


def get_i2c_bus_index(preferred=1):
    """Return an available I2C bus index, preferring the requested one."""
    indices = list_i2c_bus_indices(preferred)
    return indices[0] if indices else None


def detect_compass(bus):
    """Return the detected compass I2C address, if any."""
    for addr in (HMC5883L_ADDR, QMC5883L_ADDR):
        try:
            bus.read_byte(addr)
            return addr
        except OSError:
            continue
    return None


def init_compass(bus, addr):
    """Initialize the compass configuration for the given address."""
    if addr == HMC5883L_ADDR:
        # 8-sample average, 15 Hz, normal measurement
        bus.write_byte_data(addr, 0x00, 0x70)
        # Gain = 1090 LSB/Gauss
        bus.write_byte_data(addr, 0x01, 0x20)
        # Continuous measurement mode
        bus.write_byte_data(addr, 0x02, 0x00)
    elif addr == QMC5883L_ADDR:
        # 200 Hz, 8G, continuous
        bus.write_byte_data(addr, 0x0B, 0x01)
        bus.write_byte_data(addr, 0x09, 0x1D)


def _signed_16(value):
    return value - 65536 if value > 32767 else value


def read_compass_raw(bus, addr):
    """Read raw (x, y, z) magnetometer values."""
    if addr == HMC5883L_ADDR:
        data = bus.read_i2c_block_data(addr, 0x03, 6)
        x = _signed_16((data[0] << 8) | data[1])
        z = _signed_16((data[2] << 8) | data[3])
        y = _signed_16((data[4] << 8) | data[5])
        return x, y, z
    if addr == QMC5883L_ADDR:
        data = bus.read_i2c_block_data(addr, 0x00, 6)
        x = _signed_16((data[1] << 8) | data[0])
        y = _signed_16((data[3] << 8) | data[2])
        z = _signed_16((data[5] << 8) | data[4])
        return x, y, z
    raise ValueError("Unsupported compass address")


def heading_degrees(x, y):
    """Compute heading in degrees from X/Y."""
    heading = math.degrees(math.atan2(y, x))
    return (heading + 360.0) % 360.0


def raw_to_milligauss(x, y, z, addr):
    """Convert raw readings to milligauss (approximate)."""
    if addr == HMC5883L_ADDR:
        scale = 1000.0 / 1090.0
        return x * scale, y * scale, z * scale
    # QMC5883L scaling varies with range; keep raw counts as a best-effort.
    return float(x), float(y), float(z)


def _apply_axis_map(vec, axis_map):
    axes = {"x": 0, "y": 1, "z": 2}
    if not axis_map:
        return vec
    mapped = [0.0, 0.0, 0.0]
    for i, entry in enumerate(axis_map):
        if not entry:
            mapped[i] = vec[i]
            continue
        sign = 1.0
        axis = entry
        if entry[0] in {"+", "-"}:
            sign = -1.0 if entry[0] == "-" else 1.0
            axis = entry[1:]
        idx = axes.get(axis.lower())
        if idx is None:
            mapped[i] = vec[i]
            continue
        mapped[i] = sign * vec[idx]
    return tuple(mapped)


def _apply_calibration(vec, calibration):
    if not calibration:
        return vec
    offsets = calibration.get("offsets_mg")
    scales = calibration.get("scales")
    axis_map = calibration.get("axis_map")
    heading_offset_deg = calibration.get("heading_offset_deg")
    x, y, z = _apply_axis_map(vec, axis_map)
    if offsets and len(offsets) == 3:
        x -= offsets[0]
        y -= offsets[1]
        z -= offsets[2]
    if scales and len(scales) == 3:
        x *= scales[0]
        y *= scales[1]
        z *= scales[2]
    if heading_offset_deg:
        angle = math.radians(float(heading_offset_deg))
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        x, y = (x * cos_a - y * sin_a), (x * sin_a + y * cos_a)
    return x, y, z


class CompassReader:
    """Simple compass reader that auto-detects HMC5883L/QMC5883L."""

    def __init__(self, preferred_bus=1, calibration=None):
        self.bus_index = None
        self.bus = None
        self.addr = None
        self.calibration = calibration or {}
        bus_candidates = list_i2c_bus_indices(preferred_bus)
        if not bus_candidates:
            raise FileNotFoundError("No I2C bus found under /dev/i2c-*.")
        last_error = None
        for bus_index in bus_candidates:
            try:
                bus = SMBus(bus_index)
            except OSError as exc:
                last_error = exc
                continue
            addr = detect_compass(bus)
            if addr is not None:
                self.bus_index = bus_index
                self.bus = bus
                self.addr = addr
                init_compass(self.bus, self.addr)
                return
            bus.close()
        if last_error is not None and self.bus_index is None:
            raise RuntimeError(
                f"Failed to open any I2C bus {bus_candidates} ({last_error})."
            )
        raise RuntimeError(f"Compass not found on I2C buses {bus_candidates}.")

    def read(self):
        """Return heading (deg) and raw x/y/z."""
        x, y, z = read_compass_raw(self.bus, self.addr)
        return heading_degrees(x, y), (x, y, z)

    def read_milligauss(self):
        """Return heading (deg) and x/y/z in milligauss."""
        x, y, z = read_compass_raw(self.bus, self.addr)
        mg = raw_to_milligauss(x, y, z, self.addr)
        mg = _apply_calibration(mg, self.calibration)
        return heading_degrees(mg[0], mg[1]), mg

    def close(self):
        if self.bus is not None:
            self.bus.close()
