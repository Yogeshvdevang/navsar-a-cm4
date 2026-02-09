"""Serial GPS output helpers (NMEA/UBX) and rate limiting."""

import math
import struct
import time
import datetime as _dt

import serial

from navisar.pixhawk.fake_gps_nmea import gga_sentence, rmc_sentence, speed_course_from_enu
from navisar.pixhawk.gps_injector import FakeSatellites, hdop_from_sats


def _bytes_hex(payload):
    """Format bytes as a readable hex string."""
    return " ".join(f"{b:02X}" for b in payload)


class FakeGpsEmitter:
    """Smooth and rate-limit VPS-derived GPS output."""
    def __init__(self, send_interval_s, smooth_alpha, max_step_m):
        """Initialize emitter settings and smoothing state."""
        # Enforce 5-10 Hz update rate by clamping interval to 0.1-0.2s.
        self.send_interval_s = min(max(send_interval_s, 0.1), 0.2)
        self.smooth_alpha = max(0.0, min(1.0, float(smooth_alpha)))
        self.max_step_m = max(0.0, float(max_step_m))
        self._last_send = 0.0
        self._smoothed = None

    def smooth_position(self, x_m, y_m, z_m):
        """Apply exponential smoothing and step limiting."""
        if self._smoothed is None:
            self._smoothed = (x_m, y_m, z_m)
            return self._smoothed
        px, py, pz = self._smoothed
        nx = px + self.smooth_alpha * (x_m - px)
        ny = py + self.smooth_alpha * (y_m - py)
        nz = pz + self.smooth_alpha * (z_m - pz)
        dx = nx - px
        dy = ny - py
        dz = nz - pz
        step = float(math.hypot(math.hypot(dx, dy), dz))
        if self.max_step_m > 0.0 and step > self.max_step_m:
            scale = self.max_step_m / step
            nx = px + dx * scale
            ny = py + dy * scale
            nz = pz + dz * scale
        self._smoothed = (nx, ny, nz)
        return self._smoothed

    def ready(self, now):
        """Check if the next packet should be sent."""
        return now - self._last_send >= self.send_interval_s

    def mark_sent(self, now):
        """Record the send timestamp."""
        self._last_send = now


class NmeaSerialEmitter:
    """Emit NMEA GGA/RMC sentences over serial."""
    def __init__(
        self,
        port,
        baud,
        rate_hz,
        fix_quality,
        min_sats,
        max_sats,
        update_s,
        max_heading_delta_deg=20.0,
        raw_print=False,
    ):
        """Configure serial output and heading smoothing."""
        self.port = port
        self.baud = baud
        self.rate_hz = min(max(rate_hz, 5.0), 10.0)
        self.fix_quality = fix_quality
        self.max_heading_delta_deg = max_heading_delta_deg
        self.raw_print = raw_print
        self._ser = serial.Serial(port, baud, timeout=0)
        self._last_send = 0.0
        self._last_course = 0.0
        self._fake_sats = FakeSatellites(
            min_sats=min_sats,
            max_sats=max_sats,
            update_s=update_s,
        )

    def ready(self, now):
        """Check if the next NMEA update is due."""
        return now - self._last_send >= (1.0 / self.rate_hz)

    def send(
        self,
        lat,
        lon,
        alt_m,
        vx_e,
        vy_n,
        ekf_ok=True,
        course_deg_override=None,
        force_heading=False,
    ):
        """Generate and send NMEA messages for the current state."""
        speed_mps, course_deg = speed_course_from_enu(vx_e, vy_n)
        if course_deg_override is not None:
            desired = float(course_deg_override) % 360.0
            if force_heading:
                course_deg = desired
                self._last_course = course_deg
            else:
                delta = (desired - self._last_course + 540.0) % 360.0 - 180.0
                if abs(delta) > self.max_heading_delta_deg:
                    course_deg = (
                        self._last_course
                        + self.max_heading_delta_deg * (1 if delta > 0 else -1)
                    ) % 360.0
                else:
                    course_deg = desired
        elif speed_mps < 0.05:
            course_deg = self._last_course
        else:
            delta = (course_deg - self._last_course + 540.0) % 360.0 - 180.0
            if abs(delta) > self.max_heading_delta_deg:
                course_deg = (
                    self._last_course
                    + self.max_heading_delta_deg * (1 if delta > 0 else -1)
                ) % 360.0
        self._last_course = course_deg
        sats = self._fake_sats.update(ekf_ok=ekf_ok)
        hdop = hdop_from_sats(sats)
        gga = gga_sentence(
            lat,
            lon,
            alt_m,
            fix_quality=self.fix_quality,
            satellites=sats,
            hdop=hdop,
        )
        rmc = rmc_sentence(
            lat,
            lon,
            speed_mps,
            course_deg,
            status="A" if self.fix_quality > 0 else "V",
        )
        gga_bytes = gga.encode("ascii")
        rmc_bytes = rmc.encode("ascii")
        self._ser.write(gga_bytes)
        self._ser.write(rmc_bytes)
        if self.raw_print:
            print(f"NMEA RAW GGA: {_bytes_hex(gga_bytes)}")
            print(f"NMEA RAW RMC: {_bytes_hex(rmc_bytes)}")
        self._last_send = time.time()
        return {
            "gga": gga_bytes,
            "rmc": rmc_bytes,
            "sats": sats,
            "speed_mps": speed_mps,
            "course_deg": course_deg,
        }


class UbxSerialEmitter:
    """Emit UBX navigation messages over serial."""
    def __init__(
        self,
        port,
        baud,
        rate_hz,
        fix_type,
        min_sats,
        max_sats,
        update_s,
        max_heading_delta_deg=20.0,
        raw_print=False,
    ):
        """Configure UBX serial output and rate limiting."""
        self.port = port
        self.baud = baud
        self.rate_hz = min(max(rate_hz, 5.0), 10.0)
        self.fix_type = int(fix_type)
        self.max_heading_delta_deg = max_heading_delta_deg
        self.raw_print = raw_print
        self._ser = serial.Serial(port, baud, timeout=0)
        self._last_send = 0.0
        self._last_course = 0.0
        self._fake_sats = FakeSatellites(
            min_sats=min_sats,
            max_sats=max_sats,
            update_s=update_s,
        )

    def ready(self, now):
        """Check if the next UBX update is due."""
        return now - self._last_send >= (1.0 / self.rate_hz)

    @staticmethod
    def _ubx_checksum(msg_class, msg_id, payload):
        """Compute UBX checksum for class/id/payload."""
        ck_a = 0
        ck_b = 0

        def _update(val):
            nonlocal ck_a, ck_b
            ck_a = (ck_a + val) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF

        _update(msg_class)
        _update(msg_id)
        length = len(payload)
        _update(length & 0xFF)
        _update((length >> 8) & 0xFF)
        for byte in payload:
            _update(byte)
        return ck_a, ck_b

    @classmethod
    def _create_ubx_message(cls, msg_class, msg_id, payload):
        """Wrap a UBX payload with header and checksum."""
        length = len(payload)
        header = struct.pack("<BBBB", 0xB5, 0x62, msg_class, msg_id)
        length_bytes = struct.pack("<H", length)
        ck_a, ck_b = cls._ubx_checksum(msg_class, msg_id, payload)
        checksum = struct.pack("BB", ck_a, ck_b)
        return header + length_bytes + payload + checksum

    @classmethod
    def _create_nav_posllh(cls, lat_deg, lon_deg, alt_m, time_of_week_ms):
        """Create a UBX NAV-POSLLH message payload."""
        lat_1e7 = int(lat_deg * 1e7)
        lon_1e7 = int(lon_deg * 1e7)
        alt_mm = int(alt_m * 1000)
        h_msl_mm = alt_mm
        h_acc_mm = 2000
        v_acc_mm = 3000
        payload = struct.pack(
            "<IiiiiII",
            time_of_week_ms,
            lon_1e7,
            lat_1e7,
            alt_mm,
            h_msl_mm,
            h_acc_mm,
            v_acc_mm,
        )
        return cls._create_ubx_message(0x01, 0x02, payload)

    @classmethod
    def _create_nav_velned(
        cls,
        vel_n_mm,
        vel_e_mm,
        vel_d_mm,
        speed_mps,
        heading_deg,
        time_of_week_ms,
    ):
        """Create a UBX NAV-VELNED message payload."""
        vel_n_cm = int(vel_n_mm / 10)
        vel_e_cm = int(vel_e_mm / 10)
        vel_d_cm = int(vel_d_mm / 10)
        speed_cm = int(speed_mps * 100)
        ground_speed_cm = speed_cm
        heading_1e5 = int(heading_deg * 1e5)
        s_acc_cm = 50
        c_acc_1e5 = 5000
        payload = struct.pack(
            "<IiiiIIiII",
            time_of_week_ms,
            vel_n_cm,
            vel_e_cm,
            vel_d_cm,
            speed_cm,
            ground_speed_cm,
            heading_1e5,
            s_acc_cm,
            c_acc_1e5,
        )
        return cls._create_ubx_message(0x01, 0x12, payload)

    @classmethod
    def _create_nav_sol(cls, num_sats, time_of_week_ms, week, fix_type, p_dop_01):
        """Create a UBX NAV-SOL message payload."""
        gps_fix = int(fix_type)
        flags = 0x07
        p_acc_cm = 250
        payload = struct.pack(
            "<IihBBIiiiIIHBBII",
            time_of_week_ms,  # iTOW
            0,  # fTOW
            int(week),  # week
            gps_fix,
            flags,
            0,  # ecefX
            0,  # ecefY
            0,  # ecefZ
            p_acc_cm,  # pAcc
            0,  # ecefVX
            0,  # ecefVY
            50,  # sAcc
            int(p_dop_01),  # pDOP (0.01)
            0,  # reserved1
            num_sats,
            0,  # reserved2
        )
        return cls._create_ubx_message(0x01, 0x06, payload)

    @classmethod
    def _create_nav_status(cls, time_of_week_ms, fix_type):
        """Create a UBX NAV-STATUS message payload."""
        gps_fix = int(fix_type)
        flags = 0x01  # gpsFixOK
        fix_stat = 0x00
        flags2 = 0x07
        ttff = 0
        msss = 0
        payload = struct.pack(
            "<IBBBBII",
            time_of_week_ms,
            gps_fix,
            flags,
            fix_stat,
            flags2,
            ttff,
            msss,
        )
        return cls._create_ubx_message(0x01, 0x03, payload)

    @classmethod
    def _create_nav_dop(cls, time_of_week_ms, dop_01):
        """Create a UBX NAV-DOP message payload."""
        dop = int(dop_01)
        payload = struct.pack(
            "<IHHHHHHH",
            time_of_week_ms,
            dop,  # gDOP
            dop,  # pDOP
            dop,  # tDOP
            dop,  # vDOP
            dop,  # hDOP
            dop,  # nDOP
            dop,  # eDOP
        )
        return cls._create_ubx_message(0x01, 0x04, payload)

    @classmethod
    def _create_nav_pvt(
        cls,
        lat_deg,
        lon_deg,
        alt_m,
        vel_n_mm,
        vel_e_mm,
        vel_d_mm,
        speed_mps,
        heading_deg,
        num_sats,
        time_of_week_ms,
        now,
        fix_type,
        p_dop_01,
    ):
        """Create a UBX NAV-PVT message payload."""
        gps_fix = int(fix_type)
        valid_flags = 0x07
        fix_flags = 0x01
        flags2 = 0x00

        lat_1e7 = int(lat_deg * 1e7)
        lon_1e7 = int(lon_deg * 1e7)
        alt_mm = int(alt_m * 1000)
        h_msl_mm = int(alt_m * 1000)

        heading_1e5 = int(heading_deg * 1e5)
        payload = struct.pack(
            "<IHBBBBBBIiBBBBiiiiIIiiiiiIIHBBihH",
            time_of_week_ms,
            now.tm_year,
            now.tm_mon,
            now.tm_mday,
            now.tm_hour,
            now.tm_min,
            now.tm_sec,
            valid_flags,
            0,  # tAcc
            0,  # nano
            gps_fix,
            fix_flags,
            flags2,
            num_sats,
            lon_1e7,
            lat_1e7,
            alt_mm,
            h_msl_mm,
            500,  # hAcc
            1000,  # vAcc
            vel_n_mm,
            vel_e_mm,
            vel_d_mm,
            int(speed_mps * 1000),
            heading_1e5,
            1000,  # sAcc
            10000,  # headAcc
            int(p_dop_01),
            0,  # flags3
            0,  # reserved1
            heading_1e5,  # headVeh
            0,  # magDec
            0,  # magAcc
        )
        return cls._create_ubx_message(0x01, 0x07, payload)

    def send(
        self,
        lat,
        lon,
        alt_m,
        vx_e,
        vy_n,
        ekf_ok=True,
        course_deg_override=None,
        force_heading=False,
    ):
        """Send UBX messages for current state."""
        speed_mps, course_deg = speed_course_from_enu(vx_e, vy_n)
        if course_deg_override is not None:
            desired = float(course_deg_override) % 360.0
            if force_heading:
                course_deg = desired
                self._last_course = course_deg
            else:
                delta = (desired - self._last_course + 540.0) % 360.0 - 180.0
                if abs(delta) > self.max_heading_delta_deg:
                    course_deg = (
                        self._last_course
                        + self.max_heading_delta_deg * (1 if delta > 0 else -1)
                    ) % 360.0
                else:
                    course_deg = desired
        elif speed_mps < 0.05:
            course_deg = self._last_course
        else:
            delta = (course_deg - self._last_course + 540.0) % 360.0 - 180.0
            if abs(delta) > self.max_heading_delta_deg:
                course_deg = (
                    self._last_course
                    + self.max_heading_delta_deg * (1 if delta > 0 else -1)
                ) % 360.0
        self._last_course = course_deg
        sats = self._fake_sats.update(ekf_ok=ekf_ok)
        p_dop_01 = int(hdop_from_sats(sats) * 100)
        now = _dt.datetime.utcnow()
        gps_epoch = _dt.datetime(1980, 1, 6)
        gps_offset_s = 18.0
        total_s = (now - gps_epoch).total_seconds() + gps_offset_s
        gps_week = int(total_s // (7 * 24 * 3600))
        time_of_week_ms = int((total_s - gps_week * 7 * 24 * 3600) * 1000)
        vel_n_mm = int(vy_n * 1000)
        vel_e_mm = int(vx_e * 1000)
        vel_d_mm = 0
        pvt = self._create_nav_pvt(
            lat,
            lon,
            alt_m,
            vel_n_mm,
            vel_e_mm,
            vel_d_mm,
            speed_mps,
            course_deg,
            sats,
            time_of_week_ms,
            now.timetuple(),
            self.fix_type,
            p_dop_01,
        )
        posllh = self._create_nav_posllh(lat, lon, alt_m, time_of_week_ms)
        velned = self._create_nav_velned(
            vel_n_mm,
            vel_e_mm,
            vel_d_mm,
            speed_mps,
            course_deg,
            time_of_week_ms,
        )
        sol = self._create_nav_sol(sats, time_of_week_ms, gps_week, self.fix_type, p_dop_01)
        status = self._create_nav_status(time_of_week_ms, self.fix_type)
        dop = self._create_nav_dop(time_of_week_ms, p_dop_01)
        self._ser.write(pvt)
        self._ser.write(posllh)
        self._ser.write(velned)
        self._ser.write(sol)
        self._ser.write(status)
        self._ser.write(dop)
        if self.raw_print:
            print(f"UBX RAW NAV-PVT: {_bytes_hex(pvt)}")
            print(f"UBX RAW NAV-POSLLH: {_bytes_hex(posllh)}")
            print(f"UBX RAW NAV-VELNED: {_bytes_hex(velned)}")
            print(f"UBX RAW NAV-SOL: {_bytes_hex(sol)}")
            print(f"UBX RAW NAV-STATUS: {_bytes_hex(status)}")
            print(f"UBX RAW NAV-DOP: {_bytes_hex(dop)}")
        self._last_send = time.time()
        return {
            "pvt": pvt,
            "posllh": posllh,
            "velned": velned,
            "sol": sol,
            "status": status,
            "dop": dop,
            "sats": sats,
            "speed_mps": speed_mps,
            "course_deg": course_deg,
        }
