#!/usr/bin/env python3
"""
MTF-01 Optical Flow Sensor Reader + Web (SSE)
Serves a live 3D view in the browser and streams sensor data via /stream.
"""

import json
import queue
import serial
import struct
import sys
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

# ============================================================================
# CONFIGURATION
# ============================================================================
SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
DATA_FREQUENCY = 100  # Hz (0 = maximum speed)
HTTP_HOST = "0.0.0.0"
HTTP_PORT = 8000
HTML_FILE = Path(__file__).with_name("plotter.html")
# ============================================================================

# Protocol constants
MICOLINK_MSG_HEAD = 0xEF
MICOLINK_MSG_ID_RANGE_SENSOR = 0x51
MICOLINK_MAX_PAYLOAD_LEN = 64

# SSE clients
_clients = []
_clients_lock = threading.Lock()
_shutdown_event = threading.Event()


def _add_client(q):
    with _clients_lock:
        _clients.append(q)


def _remove_client(q):
    with _clients_lock:
        if q in _clients:
            _clients.remove(q)


def _broadcast(payload):
    data = json.dumps(payload, separators=(",", ":"))
    with _clients_lock:
        clients = list(_clients)
    for q in clients:
        try:
            q.put_nowait(data)
        except queue.Full:
            # Drop if the client is slow
            pass


class MicolinkMessage:
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


class MTF01RangeSensor:
    def __init__(self, payload_bytes):
        data = struct.unpack('<IIBBBBhhBBH', payload_bytes[:24])

        self.time_ms = data[0]
        self.distance = data[1]
        self.strength = data[2]
        self.precision = data[3]
        self.dis_status = data[4]
        self.reserved1 = data[5]
        self.flow_vel_x = data[6]
        self.flow_vel_y = data[7]
        self.flow_quality = data[8]
        self.flow_status = data[9]
        self.reserved2 = data[10]

    def to_dict(self):
        dist_valid = self.distance > 0 and self.dis_status == 1
        dist_mm = self.distance if dist_valid else 0
        dist_cm = dist_mm / 10.0 if dist_valid else 0.0
        height_m = dist_mm / 1000.0 if dist_valid else 0.0
        speed_x = self.flow_vel_x * height_m
        speed_y = self.flow_vel_y * height_m
        return {
            "ts": time.time(),
            "time_ms": self.time_ms,
            "distance_mm": dist_mm,
            "distance_cm": dist_cm,
            "height_m": height_m,
            "strength": self.strength,
            "precision": self.precision,
            "flow_vx": self.flow_vel_x,
            "flow_vy": self.flow_vel_y,
            "flow_quality": self.flow_quality,
            "flow_status": 1 if self.flow_status == 1 else 0,
            "speed_x": speed_x,
            "speed_y": speed_y,
        }

    def __str__(self):
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        dist_valid = self.distance > 0 and self.dis_status == 1
        dist_mm = self.distance if dist_valid else 0
        dist_cm = dist_mm / 10.0 if dist_valid else 0.0
        height_m = dist_mm / 1000.0 if dist_valid else 0.0
        speed_x = self.flow_vel_x * height_m
        speed_y = self.flow_vel_y * height_m
        return (
            "ts={} time_ms={} dist_mm={} dist_cm={:.1f} dist_ok={} strength={} precision={} "
            "flow_vx={} flow_vy={} flow_q={} flow_ok={} speed_x={:.2f} speed_y={:.2f}"
        ).format(
            timestamp,
            self.time_ms,
            dist_mm,
            dist_cm,
            1 if dist_valid else 0,
            self.strength,
            self.precision,
            self.flow_vel_x,
            self.flow_vel_y,
            self.flow_quality,
            1 if self.flow_status == 1 else 0,
            speed_x,
            speed_y,
        )


class MicolinkParser:
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

            calculated_checksum = self.calculate_checksum(msg)
            if calculated_checksum == msg.checksum:
                return True
            msg.reset()

        else:
            msg.reset()

        return False

    def decode_message(self):
        if self.msg.msg_id == MICOLINK_MSG_ID_RANGE_SENSOR:
            payload_bytes = bytes(self.msg.payload[:self.msg.len])
            return MTF01RangeSensor(payload_bytes)
        return None


class MTF01ReaderStreaming:
    def __init__(self, port, baudrate, data_frequency, on_data):
        self.port = port
        self.baudrate = baudrate
        self.data_frequency = data_frequency
        self.on_data = on_data
        self.ser = None
        self.running = False
        self.sequence = 0
        self.heartbeat_thread = None

    def calculate_checksum(self, data):
        return sum(data) & 0xFF

    def create_heartbeat(self):
        time_ms = int(time.time() * 1000) & 0xFFFFFFFF

        msg = bytearray([
            0xEF,
            0x01,
            0x00,
            0x01,
            self.sequence & 0xFF,
            0x0D,
        ])

        msg.extend(struct.pack('<I', time_ms))
        msg.extend(bytes([0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00]))

        checksum = self.calculate_checksum(msg)
        msg.append(checksum)

        self.sequence += 1
        return bytes(msg)

    def heartbeat_loop(self):
        while self.running:
            try:
                heartbeat = self.create_heartbeat()
                self.ser.write(heartbeat)
                self.ser.flush()
                time.sleep(0.6)
            except Exception:
                break

    def run(self):
        try:
            print(f"\n{'='*70}")
            print("MTF-01 READER + WEB")
            print(f"{'='*70}")
            print(f"Port: {self.port}")
            print(f"Baud: {self.baudrate}")
            print(f"Frequency: {self.data_frequency} Hz" if self.data_frequency > 0 else "Frequency: MAXIMUM")
            print()

            self.ser = serial.Serial(
                self.port, self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )

            print("✓ Port opened")

            self.ser.setDTR(True)
            self.ser.setRTS(True)
            time.sleep(0.1)

            print("✓ DTR/RTS set")

            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

            print("✓ Sending initial heartbeat...")
            for _ in range(3):
                heartbeat = self.create_heartbeat()
                self.ser.write(heartbeat)
                self.ser.flush()
                time.sleep(0.1)

            time.sleep(0.3)
            self.ser.reset_input_buffer()

            self.running = True
            self.heartbeat_thread = threading.Thread(target=self.heartbeat_loop, daemon=True)
            self.heartbeat_thread.start()

            print("\n📊 Reading sensor data and streaming to /stream ...\n")

            delay = (1.0 / self.data_frequency) if self.data_frequency > 0 else 0

            parser = MicolinkParser()
            last_send_time = 0
            data_received = False
            start_time = time.time()

            while not _shutdown_event.is_set():
                if self.ser.in_waiting > 0:
                    byte = self.ser.read(1)
                    if len(byte) > 0:
                        if not data_received:
                            data_received = True
                            elapsed = time.time() - start_time
                            print(f"✓ Raw data detected after {elapsed:.2f}s")
                            print("✓ Synchronizing...\n")

                        if parser.parse_char(byte[0]):
                            sensor_data = parser.decode_message()
                            if sensor_data:
                                current_time = time.time()
                                if delay == 0 or (current_time - last_send_time) >= delay:
                                    self.on_data(sensor_data)
                                    last_send_time = current_time
                else:
                    if not data_received and (time.time() - start_time) > 5:
                        print("\n⚠️  No data after 5 seconds")
                        print("Trying alternate DTR/RTS...\n")
                        self.ser.setDTR(False)
                        self.ser.setRTS(False)
                        time.sleep(0.1)
                        self.ser.reset_input_buffer()
                        start_time = time.time()

        except serial.SerialException as e:
            print(f"\n❌ Serial Error: {e}")
            print("\nTroubleshooting:")
            print("• Close MicoAssistant")
            print(f"• Check port {self.port}")
            print("• Verify connection")

        except KeyboardInterrupt:
            print("\n\n✋ Stopped by user")

        finally:
            self.running = False
            if self.heartbeat_thread:
                self.heartbeat_thread.join(timeout=1)
            if self.ser and self.ser.is_open:
                self.ser.setDTR(False)
                self.ser.setRTS(False)
                self.ser.close()
                print("✓ Port closed")


class WebHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/" or self.path == "/plotter.html":
            if not HTML_FILE.exists():
                self.send_response(404)
                self.end_headers()
                self.wfile.write(b"Missing optical_flow.html")
                return

            data = HTML_FILE.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
            return

        if self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.end_headers()

            q = queue.Queue(maxsize=100)
            _add_client(q)

            try:
                self.wfile.write(b": connected\n\n")
                self.wfile.flush()
                while not _shutdown_event.is_set():
                    try:
                        data = q.get(timeout=0.5)
                    except queue.Empty:
                        self.wfile.write(b": ping\n\n")
                        self.wfile.flush()
                        continue
                    self.wfile.write(b"data: " + data.encode("utf-8") + b"\n\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                _remove_client(q)
            return

        self.send_response(404)
        self.end_headers()

    def log_message(self, format, *args):
        # Silence default logging
        return


def main():
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        port = SERIAL_PORT

    if len(sys.argv) > 2:
        baudrate = int(sys.argv[2])
    else:
        baudrate = BAUDRATE

    if len(sys.argv) > 3:
        frequency = float(sys.argv[3])
    else:
        frequency = DATA_FREQUENCY

    server = ThreadingHTTPServer((HTTP_HOST, HTTP_PORT), WebHandler)

    def on_data(sensor_data):
        payload = sensor_data.to_dict()
        _broadcast(payload)

    reader = MTF01ReaderStreaming(port, baudrate, frequency, on_data=on_data)

    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    print(f"\n✅ Web server running at http://localhost:{HTTP_PORT}")
    print(f"✅ Streaming endpoint at http://localhost:{HTTP_PORT}/stream\n")

    try:
        reader.run()
    finally:
        _shutdown_event.set()
        server.shutdown()
        server.server_close()


if __name__ == "__main__":
    main()
