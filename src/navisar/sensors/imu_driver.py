"""I2C IMU driver helpers for MPU6050 and ICM42688P."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

try:
    from smbus2 import SMBus
except ModuleNotFoundError:  # pragma: no cover - hardware dependency
    SMBus = None


GRAVITY_MPS2 = 9.80665
READ_TIMEOUT_S = 0.002
HEALTH_WINDOW_S = 0.1


@dataclass(frozen=True)
class ImuSample:
    """Single IMU sample in SI units plus derived tilt."""

    gyro_x: float
    gyro_y: float
    gyro_z: float
    accel_x: float
    accel_y: float
    accel_z: float
    roll_rad: float
    pitch_rad: float
    timestamp_monotonic: float


class ImuDriver:
    """Read IMU samples over I2C with health tracking and last-good caching."""

    _MPU6050 = {
        "accel_reg": 0x3B,
        "gyro_reg": 0x43,
        "init": (
            (0x6B, 0x00),  # PWR_MGMT_1
            (0x1C, 0x00),  # ACCEL_CONFIG +/-2g
            (0x1B, 0x08),  # GYRO_CONFIG +/-500 deg/s
        ),
        "accel_scale": GRAVITY_MPS2 / 16384.0,
        "gyro_scale_rad": math.radians(1.0 / 65.5),
    }
    _ICM42688P = {
        "accel_reg": 0x1F,
        "gyro_reg": 0x25,
        "init": (
            (0x4E, 0x0F),  # PWR_MGMT0
        ),
        "accel_scale": GRAVITY_MPS2 / 2048.0,
        "gyro_scale_rad": math.radians(1.0 / 131.0),
    }

    def __init__(self, bus_number: int, address: int, chip_type: str):
        self.bus_number = int(bus_number)
        self.address = int(address)
        self.chip_type = str(chip_type).strip().upper()
        if self.chip_type not in {"MPU6050", "ICM42688P"}:
            raise ValueError(f"Unsupported IMU chip '{chip_type}'")
        self._cfg = (
            self._MPU6050 if self.chip_type == "MPU6050" else self._ICM42688P
        )
        self._bus = None
        self._last_sample = None
        self._last_read_ok_monotonic = None
        self._last_error_monotonic = None
        self._read_attempts = 0
        self._read_successes = 0

    def init(self):
        """Open the bus and configure the sensor."""
        if SMBus is None:
            raise ModuleNotFoundError(
                "Missing dependency 'smbus2'. Install project requirements first."
            )
        if self._bus is None:
            self._bus = SMBus(self.bus_number)
        for register, value in self._cfg["init"]:
            self._bus.write_byte_data(self.address, register, value)
        time.sleep(0.02)

    def close(self):
        """Close the underlying I2C bus."""
        if self._bus is None:
            return
        try:
            self._bus.close()
        finally:
            self._bus = None

    @staticmethod
    def _to_int16(msb: int, lsb: int) -> int:
        value = (msb << 8) | lsb
        if value & 0x8000:
            value -= 0x10000
        return value

    def _read_vec3(self, register: int):
        raw = self._bus.read_i2c_block_data(self.address, register, 6)
        return (
            self._to_int16(raw[0], raw[1]),
            self._to_int16(raw[2], raw[3]),
            self._to_int16(raw[4], raw[5]),
        )

    def _build_sample(self, accel_raw, gyro_raw, now_monotonic: float) -> ImuSample:
        accel_scale = self._cfg["accel_scale"]
        gyro_scale = self._cfg["gyro_scale_rad"]
        accel_x = accel_raw[0] * accel_scale
        accel_y = accel_raw[1] * accel_scale
        accel_z = accel_raw[2] * accel_scale
        gyro_x = gyro_raw[0] * gyro_scale
        gyro_y = gyro_raw[1] * gyro_scale
        gyro_z = gyro_raw[2] * gyro_scale
        roll_rad = math.atan2(accel_y, accel_z)
        pitch_rad = math.atan2(-accel_x, math.hypot(accel_y, accel_z))
        return ImuSample(
            gyro_x=gyro_x,
            gyro_y=gyro_y,
            gyro_z=gyro_z,
            accel_x=accel_x,
            accel_y=accel_y,
            accel_z=accel_z,
            roll_rad=roll_rad,
            pitch_rad=pitch_rad,
            timestamp_monotonic=now_monotonic,
        )

    def read(self) -> ImuSample:
        """Read one IMU sample and update health counters."""
        if self._bus is None:
            raise RuntimeError("IMU driver not initialized.")
        self._read_attempts += 1
        start = time.monotonic()
        try:
            accel_raw = self._read_vec3(self._cfg["accel_reg"])
            gyro_raw = self._read_vec3(self._cfg["gyro_reg"])
        except Exception:
            self._last_error_monotonic = time.monotonic()
            raise
        end = time.monotonic()
        if end - start > READ_TIMEOUT_S:
            self._last_error_monotonic = end
            raise TimeoutError(
                f"IMU read exceeded {READ_TIMEOUT_S * 1000.0:.1f} ms budget"
            )
        sample = self._build_sample(accel_raw, gyro_raw, end)
        self._last_sample = sample
        self._last_read_ok_monotonic = end
        self._read_successes += 1
        return sample

    def last_sample(self):
        """Return the last successful sample, if any."""
        return self._last_sample

    def is_healthy(self) -> bool:
        """Return True when a recent successful read exists."""
        if self._last_read_ok_monotonic is None:
            return False
        return (time.monotonic() - self._last_read_ok_monotonic) <= HEALTH_WINDOW_S

    def success_rate(self) -> float:
        """Return cumulative read success rate."""
        if self._read_attempts <= 0:
            return 1.0
        return float(self._read_successes) / float(self._read_attempts)
