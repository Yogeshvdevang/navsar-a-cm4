import time
import serial
from serial.tools import list_ports

# Set PORT = None to scan all detected ports.
PORT = None
# PORT = "/dev/ttyUSB0"  # Linux/macOS example
# Common GPS baud rates; adjust if needed.
BAUDS = [4800, 9600, 19200, 38400, 57600, 115200]
PROBE_SECONDS = 3

def detect_port():
    ports = list(list_ports.comports())
    if not ports:
        raise SystemExit("No serial ports found. Set PORT manually.")
    print("Detected serial ports:")
    for idx, port in enumerate(ports, start=1):
        print(f"  {idx}) {port.device} - {port.description}")
    preferred = []
    fallback = []
    for port in ports:
        desc = (port.description or "").lower()
        if "usb" in desc or "uart" in desc or "cp210" in desc or "ch340" in desc:
            preferred.append(port.device)
        else:
            fallback.append(port.device)
    return preferred + fallback


def read_nmea(ser, seconds):
    end_time = time.time() + seconds
    saw_data = False
    while time.time() < end_time:
        line = ser.readline().decode(errors="ignore").strip()
        if line.startswith("$"):
            saw_data = True
            print(line)
    return saw_data


def try_read(port, baud, seconds):
    try:
        with serial.Serial(port, baud, timeout=1) as ser:
            print(f"Probing on {port} @ {baud} baud for {seconds}s...")
            return read_nmea(ser, seconds)
    except serial.SerialException as exc:
        print(f"Failed to open {port} @ {baud}: {exc}")
    return False


def find_port_and_baud():
    ports = detect_port() if PORT is None else [PORT]
    for port in ports:
        for baud in BAUDS:
            if try_read(port, baud, PROBE_SECONDS):
                return port, baud
    return None

choice = find_port_and_baud()
if not choice:
    raise SystemExit("No NMEA data received. Check wiring, port, and baud rate.")

port, baud = choice
print(f"Found NMEA on {port} @ {baud}")
