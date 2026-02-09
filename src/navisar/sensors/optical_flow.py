"""MTF-01 optical flow sensor reader utilities."""

import struct
import threading
import time
from datetime import datetime

import serial

MICOLINK_MSG_HEAD = 0xEF
MICOLINK_MSG_ID_RANGE_SENSOR = 0x51
MICOLINK_MAX_PAYLOAD_LEN = 64


class MicolinkMessage:
    """Hold and parse a Micolink message."""

    def __init__(self):
        self.head = 0
        self.dev_id = 0
        self.sys_id = 0
        self.msg_id = 0
        self.seq = 0
        self.len = 0
        self.payload = bytearray(MICOLINK_MAX_PAYLOAD_LEN)
        self.checksum = 0
        self.status = 0
        self.payload_cnt = 0

    def reset(self):
        self.status = 0
        self.payload_cnt = 0


class OpticalFlowSample:
    """Parsed optical-flow payload with derived values."""

    def __init__(self, payload_bytes):
        data = struct.unpack("<IIBBBBhhBBH", payload_bytes[:24])
        self.time_ms = data[0]
        self.distance_mm = data[1]
        self.strength = data[2]
        self.precision = data[3]
        self.dis_status = data[4]
        self.reserved1 = data[5]
        self.flow_vx = data[6]
        self.flow_vy = data[7]
        self.flow_quality = data[8]
        self.flow_status = data[9]
        self.reserved2 = data[10]
        self.timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

        dist_valid = self.distance_mm > 0 and self.dis_status == 1
        self.dist_ok = 1 if dist_valid else 0
        self.dist_cm = self.distance_mm / 10.0 if dist_valid else 0.0
        height_m = self.distance_mm / 1000.0 if dist_valid else 0.0
        self.speed_x = self.flow_vx * height_m
        self.speed_y = self.flow_vy * height_m
        self.flow_ok = 1 if self.flow_status == 1 else 0

    def to_dict(self):
        return {
            "ts": self.timestamp,
            "time_ms": self.time_ms,
            "dist_mm": self.distance_mm,
            "dist_cm": self.dist_cm,
            "dist_ok": self.dist_ok,
            "strength": self.strength,
            "precision": self.precision,
            "flow_vx": self.flow_vx,
            "flow_vy": self.flow_vy,
            "flow_q": self.flow_quality,
            "flow_ok": self.flow_ok,
            "speed_x": self.speed_x,
            "speed_y": self.speed_y,
        }

    def format_line(self, index):
        return (
            "[{idx:05d}] ts={ts} time_ms={time_ms} dist_mm={dist_mm} dist_cm={dist_cm:.1f} "
            "dist_ok={dist_ok} strength={strength} precision={precision} flow_vx={flow_vx} "
            "flow_vy={flow_vy} flow_q={flow_q} flow_ok={flow_ok} speed_x={speed_x:.2f} "
            "speed_y={speed_y:.2f}"
        ).format(idx=index, **self.to_dict())


class MicolinkParser:
    """Parser for the Micolink protocol."""

    def __init__(self):
        self.msg = MicolinkMessage()

    def calculate_checksum(self, msg):
        checksum = msg.head + msg.dev_id + msg.sys_id + msg.msg_id + msg.seq + msg.len
        for i in range(msg.len):
            checksum += msg.payload[i]
        return checksum & 0xFF

    def parse_char(self, data):
        msg = self.msg
        if msg.status == 0:
            if data == MICOLINK_MSG_HEAD:
                msg.head = data
                msg.status += 1
        elif msg.status == 1:
            msg.dev_id = data
            msg.status += 1
        elif msg.status == 2:
            msg.sys_id = data
            msg.status += 1
        elif msg.status == 3:
            msg.msg_id = data
            msg.status += 1
        elif msg.status == 4:
            msg.seq = data
            msg.status += 1
        elif msg.status == 5:
            msg.len = data
            if msg.len == 0:
                msg.status += 2
            elif msg.len > MICOLINK_MAX_PAYLOAD_LEN:
                msg.reset()
            else:
                msg.status += 1
        elif msg.status == 6:
            msg.payload[msg.payload_cnt] = data
            msg.payload_cnt += 1
            if msg.payload_cnt == msg.len:
                msg.payload_cnt = 0
                msg.status += 1
        elif msg.status == 7:
            msg.checksum = data
            msg.status = 0
            calculated = self.calculate_checksum(msg)
            if calculated == msg.checksum:
                return True
            msg.reset()
        else:
            msg.reset()
        return False

    def decode_message(self):
        if self.msg.msg_id == MICOLINK_MSG_ID_RANGE_SENSOR:
            payload_bytes = bytes(self.msg.payload[: self.msg.len])
            return OpticalFlowSample(payload_bytes)
        return None


class MTF01OpticalFlowReader:
    """Threaded reader for MTF-01 optical flow sensor."""

    def __init__(
        self,
        port,
        baudrate=115200,
        data_frequency=100.0,
        heartbeat_interval_s=0.6,
        print_enabled=False,
        on_sample=None,
        max_flow_raw=None,
    ):
        self.port = port
        self.baudrate = int(baudrate)
        self.data_frequency = float(data_frequency)
        self.heartbeat_interval_s = float(heartbeat_interval_s)
        self.print_enabled = bool(print_enabled)
        self.on_sample = on_sample
        self.max_flow_raw = (
            None if max_flow_raw is None else float(max_flow_raw)
        )

        self._ser = None
        self._running = False
        self._sequence = 0
        self._thread = None
        self._lock = threading.Lock()
        self._last_sample = None
        self._last_error = None
        self._msg_count = 0

    def _calculate_checksum(self, data):
        return sum(data) & 0xFF

    def _create_heartbeat(self):
        time_ms = int(time.time() * 1000) & 0xFFFFFFFF
        msg = bytearray(
            [
                0xEF,
                0x01,
                0x00,
                0x01,
                self._sequence & 0xFF,
                0x0D,
            ]
        )
        msg.extend(struct.pack("<I", time_ms))
        msg.extend(bytes([0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00]))
        msg.append(self._calculate_checksum(msg))
        self._sequence += 1
        return bytes(msg)

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._ser and self._ser.is_open:
            try:
                self._ser.setDTR(False)
                self._ser.setRTS(False)
                self._ser.close()
            except Exception:
                pass

    def get_latest(self):
        with self._lock:
            return self._last_sample

    def get_last_error(self):
        with self._lock:
            return self._last_error

    def _store_sample(self, sample):
        with self._lock:
            self._last_sample = sample

    def _run(self):
        try:
            self._ser = serial.Serial(
                self.port,
                self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            )
            self._ser.setDTR(True)
            self._ser.setRTS(True)
            time.sleep(0.1)
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()

            for _ in range(3):
                self._ser.write(self._create_heartbeat())
                self._ser.flush()
                time.sleep(0.1)

            time.sleep(0.3)
            self._ser.reset_input_buffer()

            parser = MicolinkParser()
            last_emit_time = 0.0
            delay = (1.0 / self.data_frequency) if self.data_frequency > 0 else 0.0

            while self._running:
                if self._ser.in_waiting > 0:
                    byte = self._ser.read(1)
                    if byte and parser.parse_char(byte[0]):
                        sample = parser.decode_message()
                        if sample is None:
                            continue
                        if (
                            self.max_flow_raw is not None
                            and (
                                abs(sample.flow_vx) >= self.max_flow_raw
                                or abs(sample.flow_vy) >= self.max_flow_raw
                            )
                        ):
                            sample.flow_quality = 0
                            sample.flow_status = 0
                            sample.flow_ok = 0
                            sample.speed_x = 0.0
                            sample.speed_y = 0.0
                        now = time.time()
                        if delay == 0 or (now - last_emit_time) >= delay:
                            last_emit_time = now
                            self._msg_count += 1
                            self._store_sample(sample)
                            if self.print_enabled:
                                print(sample.format_line(self._msg_count))
                            if self.on_sample is not None:
                                try:
                                    self.on_sample(sample)
                                except Exception:
                                    pass
                else:
                    time.sleep(0.01)
        except Exception as exc:
            with self._lock:
                self._last_error = exc
        finally:
            self._running = False
            if self._ser and self._ser.is_open:
                try:
                    self._ser.setDTR(False)
                    self._ser.setRTS(False)
                    self._ser.close()
                except Exception:
                    pass
