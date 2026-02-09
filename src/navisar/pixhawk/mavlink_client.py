"""MAVLink wrapper for Pixhawk IO (GPS, attitude, odometry)."""

import time

import serial

from pymavlink import mavutil


class MavlinkInterface:
    """Thin wrapper around pymavlink for Pixhawk IO."""
    def __init__(
        self,
        device,
        baud=115200,
        heartbeat_timeout=5.0,
        source_system=200,
        source_component=None,
        rangefinder_component=None,
    ):
        """Connect to the MAVLink device and wait for heartbeat."""
        self.device = device
        self.baud = baud
        if source_component is None:
            source_component = getattr(
                mavutil.mavlink, "MAV_COMP_ID_PERIPHERAL", 193
            )
        if rangefinder_component is None:
            rangefinder_component = getattr(
                mavutil.mavlink, "MAV_COMP_ID_RANGEFINDER", 173
            )
        self.master = mavutil.mavlink_connection(
            device,
            baud=baud,
            source_system=int(source_system),
            source_component=int(source_component),
        )
        self._range_master = None
        if int(rangefinder_component) != int(source_component):
            self._range_master = mavutil.mavlink_connection(
                device,
                baud=baud,
                source_system=int(source_system),
                source_component=int(rangefinder_component),
            )
        self._last_attitude = None
        self._last_error_time = 0.0
        self._wait_heartbeat(heartbeat_timeout)

    def _wait_heartbeat(self, timeout):
        """Block until a heartbeat is received or timeout occurs."""
        try:
            self.master.wait_heartbeat(timeout=timeout)
        except Exception as exc:
            raise RuntimeError("Failed to receive MAVLink heartbeat") from exc

    def recv_distance_sensor(self):
        """Receive a non-blocking DISTANCE_SENSOR message."""
        return self.master.recv_match(type="DISTANCE_SENSOR", blocking=False)

    def request_message_interval(self, msg_id, rate_hz):
        """Request periodic MAVLink messages by ID."""
        if rate_hz <= 0:
            return
        # MAV_CMD_SET_MESSAGE_INTERVAL expects microseconds between messages.
        interval_us = int(1_000_000 / rate_hz)
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            interval_us,
            0,
            0,
            0,
            0,
            0,
        )

    def recv_gps(self):
        """Receive and parse the latest GPS message."""
        msg = self.recv_gps_raw()
        return self._parse_gps_msg(msg)

    def recv_gps_raw(self):
        """Receive the raw GPS message without parsing."""
        return self.master.recv_match(
            type=["GPS_RAW_INT", "GLOBAL_POSITION_INT"],
            blocking=False,
        )

    def recv_attitude(self):
        """Receive an ATTITUDE message and cache it."""
        try:
            msg = self.master.recv_match(type="ATTITUDE", blocking=False)
        except serial.SerialException as exc:
            now = time.time()
            if now - self._last_error_time > 2.0:
                self._last_error_time = now
                print(f"Warning: MAVLink serial error ({exc})")
            return None
        if msg is None:
            return None
        att = {
            "roll": float(msg.roll),
            "pitch": float(msg.pitch),
            "yaw": float(msg.yaw),
            "roll_rate": float(msg.rollspeed),
            "pitch_rate": float(msg.pitchspeed),
            "yaw_rate": float(msg.yawspeed),
            "time_s": time.time(),
        }
        self._last_attitude = att
        return att

    def recv_imu(self):
        """Receive a HIGHRES_IMU or RAW_IMU message and return parsed accel/gyro data."""
        try:
            msg = self.master.recv_match(type=["HIGHRES_IMU", "RAW_IMU"], blocking=False)
        except serial.SerialException as exc:
            now = time.time()
            if now - self._last_error_time > 2.0:
                self._last_error_time = now
                print(f"Warning: MAVLink serial error ({exc})")
            return None
        if msg is None:
            return None
        msg_type = msg.get_type()
        if msg_type == "RAW_IMU":
            time_usec = getattr(msg, "time_usec", None)
            time_s = time_usec * 1e-6 if time_usec else time.time()
            return {
                "ax": float(msg.xacc) * 9.80665 / 1000.0,
                "ay": float(msg.yacc) * 9.80665 / 1000.0,
                "az": float(msg.zacc) * 9.80665 / 1000.0,
                "gx": float(msg.xgyro) / 1000.0,
                "gy": float(msg.ygyro) / 1000.0,
                "gz": float(msg.zgyro) / 1000.0,
                "time_s": float(time_s),
            }
        time_usec = getattr(msg, "time_usec", None)
        time_s = time_usec * 1e-6 if time_usec else time.time()
        return {
            "ax": float(msg.xacc),
            "ay": float(msg.yacc),
            "az": float(msg.zacc),
            "gx": float(msg.xgyro),
            "gy": float(msg.ygyro),
            "gz": float(msg.zgyro),
            "time_s": float(time_s),
        }

    def recv_barometer(self):
        """Receive a barometer-related message."""
        try:
            msg = self.master.recv_match(
                type=["SCALED_PRESSURE", "SCALED_PRESSURE2", "SCALED_PRESSURE3", "VFR_HUD"],
                blocking=False,
            )
        except serial.SerialException as exc:
            now = time.time()
            if now - self._last_error_time > 2.0:
                self._last_error_time = now
                print(f"Warning: MAVLink serial error ({exc})")
            return None
        if msg is None:
            return None
        msg_type = msg.get_type()
        if msg_type == "VFR_HUD":
            return {
                "alt_m": float(msg.alt),
                "press_hpa": None,
                "time_s": time.time(),
            }
        press_abs = getattr(msg, "press_abs", None)
        press_hpa = float(press_abs) if press_abs is not None else None
        return {
            "alt_m": None,
            "press_hpa": press_hpa,
            "time_s": time.time(),
        }

    def get_last_attitude(self):
        """Return the last cached attitude, if any."""
        return self._last_attitude

    def recv_gps_with_raw(self):
        """Return parsed GPS data plus the raw message."""
        msg = self.recv_gps_raw()
        return self._parse_gps_msg(msg), msg

    def send_gps_input(
        self,
        lat,
        lon,
        alt_m,
        fix_type=3,
        satellites_visible=10,
        vn=0.0,
        ve=0.0,
        vd=0.0,
        speed_accuracy=0.5,
        horiz_accuracy=1.0,
        vert_accuracy=1.0,
        time_usec=None,
        yaw_cdeg=None,
        ignore_flags=None,
    ):
        """Send GPS_INPUT data to Pixhawk."""
        if time_usec is None:
            time_usec = int(time.time() * 1_000_000)
        unix_time_s = time_usec / 1_000_000
        gps_epoch_s = 315964800.0
        gps_time_s = max(0.0, unix_time_s - gps_epoch_s)
        time_week = int(gps_time_s // (7 * 24 * 3600))
        time_week_ms = int((gps_time_s % (7 * 24 * 3600)) * 1000.0)
        # MAVLink expects lat/lon in 1e7 degrees and altitude in meters.
        gps_id = 0
        ignore_flags = 0 if ignore_flags is None else int(ignore_flags)
        def _send_without_yaw():
            self.master.mav.gps_input_send(
                time_usec,
                gps_id,
                ignore_flags,
                time_week_ms,
                time_week,
                fix_type,
                int(lat * 1e7),
                int(lon * 1e7),
                float(alt_m),
                float(horiz_accuracy),
                float(vert_accuracy),
                float(vn),
                float(ve),
                float(vd),
                float(speed_accuracy),
                float(horiz_accuracy),
                float(vert_accuracy),
                satellites_visible,
            )
            return self.master.mav.gps_input_encode(
                time_usec,
                gps_id,
                ignore_flags,
                time_week_ms,
                time_week,
                fix_type,
                int(lat * 1e7),
                int(lon * 1e7),
                float(alt_m),
                float(horiz_accuracy),
                float(vert_accuracy),
                float(vn),
                float(ve),
                float(vd),
                float(speed_accuracy),
                float(horiz_accuracy),
                float(vert_accuracy),
                satellites_visible,
            )

        def _send_with_yaw(yaw_value):
            mavutil.mavlink.MAVLINK20 = 1
            self.master.mav.gps_input_send(
                time_usec,
                gps_id,
                ignore_flags,
                time_week_ms,
                time_week,
                fix_type,
                int(lat * 1e7),
                int(lon * 1e7),
                float(alt_m),
                float(horiz_accuracy),
                float(vert_accuracy),
                float(vn),
                float(ve),
                float(vd),
                float(speed_accuracy),
                float(horiz_accuracy),
                float(vert_accuracy),
                satellites_visible,
                int(yaw_value),
            )
            return self.master.mav.gps_input_encode(
                time_usec,
                gps_id,
                ignore_flags,
                time_week_ms,
                time_week,
                fix_type,
                int(lat * 1e7),
                int(lon * 1e7),
                float(alt_m),
                float(horiz_accuracy),
                float(vert_accuracy),
                float(vn),
                float(ve),
                float(vd),
                float(speed_accuracy),
                float(horiz_accuracy),
                float(vert_accuracy),
                satellites_visible,
                int(yaw_value),
            )

        msg = None
        if yaw_cdeg is None:
            msg = _send_without_yaw()
        else:
            try:
                msg = _send_with_yaw(yaw_cdeg)
            except TypeError:
                if not getattr(self, "_warned_gps_input_yaw", False):
                    print("Warning: MAVLink GPS_INPUT yaw not supported; sending without yaw.")
                    self._warned_gps_input_yaw = True
                msg = _send_without_yaw()
        return {
            "raw": msg.pack(self.master.mav),
            "time_usec": time_usec,
            "gps_id": gps_id,
            "ignore_flags": ignore_flags,
            "time_week_ms": time_week_ms,
            "time_week": time_week,
            "fix_type": fix_type,
            "lat": lat,
            "lon": lon,
            "alt_m": alt_m,
            "hdop": float(horiz_accuracy),
            "vdop": float(vert_accuracy),
            "vn": float(vn),
            "ve": float(ve),
            "vd": float(vd),
            "speed_accuracy": float(speed_accuracy),
            "horiz_accuracy": float(horiz_accuracy),
            "vert_accuracy": float(vert_accuracy),
            "satellites_visible": int(satellites_visible),
            "yaw_cdeg": int(yaw_cdeg) if yaw_cdeg is not None else None,
        }

    def send_odometry(
        self,
        x,
        y,
        z,
        q,
        vx,
        vy,
        vz,
        roll_rate=0.0,
        pitch_rate=0.0,
        yaw_rate=0.0,
        time_usec=None,
        frame=mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        pose_covariance=None,
        velocity_covariance=None,
        reset_counter=0,
        estimator_type=mavutil.mavlink.MAV_ESTIMATOR_TYPE_VISION,
        quality=100,
    ):
        """Send MAVLink ODOMETRY message."""
        if time_usec is None:
            time_usec = int(time.time() * 1_000_000)
        if pose_covariance is None:
            # Default covariance reflects modest confidence in VO estimates.
            pose_covariance = self._diag_covariance(
                [0.04, 0.04, 0.09, 0.03, 0.03, 0.03]
            )
        if velocity_covariance is None:
            velocity_covariance = self._diag_covariance(
                [0.25, 0.25, 0.25, 0.09, 0.09, 0.09]
            )
        self.master.mav.odometry_send(
            time_usec,
            frame,
            frame,
            x,
            y,
            z,
            q,
            vx,
            vy,
            vz,
            roll_rate,
            pitch_rate,
            yaw_rate,
            pose_covariance,
            velocity_covariance,
            int(reset_counter),
            int(estimator_type),
            int(quality),
        )

    def send_statustext(self, text, severity=mavutil.mavlink.MAV_SEVERITY_WARNING):
        """Send a STATUSTEXT message for GCS display."""
        if not text:
            return
        payload = text.encode("ascii", "ignore")[:50]
        self.master.mav.statustext_send(int(severity), payload)

    def send_compass(self, x_mg, y_mg, z_mg, time_boot_ms=None):
        """Send compass data as SCALED_IMU2 (mag only)."""
        if time_boot_ms is None:
            # MAVLink SCALED_IMU2 expects uint32 time_boot_ms.
            time_boot_ms = int(time.time() * 1000) % (2**32)
        else:
            time_boot_ms = int(time_boot_ms) % (2**32)
        self.master.mav.scaled_imu2_send(
            int(time_boot_ms),
            0,
            0,
            0,
            0,
            0,
            0,
            int(x_mg),
            int(y_mg),
            int(z_mg),
        )

    def send_distance_sensor(
        self,
        distance_m,
        min_distance_m=0.01,
        max_distance_m=8.0,
        orientation=mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270,
        sensor_type=mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
        sensor_id=0,
        covariance=255,
        time_boot_ms=None,
    ):
        """Send DISTANCE_SENSOR message."""
        master = self._range_master or self.master
        if time_boot_ms is None:
            time_boot_ms = int(time.time() * 1000) % (2**32)
        distance_cm = int(max(0.0, float(distance_m)) * 100)
        min_cm = int(max(0.0, float(min_distance_m)) * 100)
        max_cm = int(max(0.0, float(max_distance_m)) * 100)
        master.mav.distance_sensor_send(
            int(time_boot_ms),
            int(min_cm),
            int(max_cm),
            int(distance_cm),
            int(sensor_type),
            int(sensor_id),
            int(orientation),
            int(covariance),
        )
        return {
            "time_boot_ms": int(time_boot_ms),
            "min_cm": int(min_cm),
            "max_cm": int(max_cm),
            "distance_cm": int(distance_cm),
            "sensor_type": int(sensor_type),
            "sensor_id": int(sensor_id),
            "orientation": int(orientation),
            "covariance": int(covariance),
        }

    def send_optical_flow_rad(
        self,
        integrated_x,
        integrated_y,
        integration_time_us,
        distance_m,
        quality,
        time_usec=None,
        sensor_id=0,
        integrated_xgyro=0.0,
        integrated_ygyro=0.0,
        integrated_zgyro=0.0,
        temperature=0,
        time_delta_distance_us=None,
    ):
        """Send OPTICAL_FLOW_RAD message."""
        if time_usec is None:
            time_usec = int(time.time() * 1_000_000)
        if time_delta_distance_us is None:
            time_delta_distance_us = int(integration_time_us)
        self.master.mav.optical_flow_rad_send(
            int(time_usec),
            int(sensor_id),
            int(integration_time_us),
            float(integrated_x),
            float(integrated_y),
            float(integrated_xgyro),
            float(integrated_ygyro),
            float(integrated_zgyro),
            int(temperature),
            int(quality),
            int(time_delta_distance_us),
            float(distance_m),
        )
        return {
            "time_usec": int(time_usec),
            "sensor_id": int(sensor_id),
            "integration_time_us": int(integration_time_us),
            "integrated_x": float(integrated_x),
            "integrated_y": float(integrated_y),
            "integrated_xgyro": float(integrated_xgyro),
            "integrated_ygyro": float(integrated_ygyro),
            "integrated_zgyro": float(integrated_zgyro),
            "temperature": int(temperature),
            "quality": int(quality),
            "time_delta_distance_us": int(time_delta_distance_us),
            "distance_m": float(distance_m),
        }

    @staticmethod
    def _parse_gps_msg(msg):
        """Parse MAVLink GPS messages into a dict."""
        if msg is None:
            return None
        msg_type = msg.get_type()
        if msg_type == "GPS_RAW_INT":
            if msg.lat == 0 and msg.lon == 0:
                return None
            return {
                "lat": msg.lat / 1e7,
                "lon": msg.lon / 1e7,
                "alt_m": msg.alt / 1000.0,
                "fix_type": msg.fix_type,
            }
        if msg_type == "GLOBAL_POSITION_INT":
            if msg.lat == 0 and msg.lon == 0:
                return None
            return {
                "lat": msg.lat / 1e7,
                "lon": msg.lon / 1e7,
                "alt_m": msg.alt / 1000.0,
                "fix_type": None,
            }
        return None

    @staticmethod
    def _diag_covariance(diag):
        """Expand a 6D diagonal covariance into MAVLink format."""
        if len(diag) != 6:
            raise ValueError("Expected 6 diagonal covariance values.")
        cov = [0.0] * 21
        diag_indices = [0, 6, 11, 15, 18, 20]
        for idx, value in zip(diag_indices, diag):
            cov[idx] = float(value)
        return cov
