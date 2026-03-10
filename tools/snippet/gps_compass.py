#!/usr/bin/env python3

import glob
import math
import time

import serial

try:
    from smbus2 import SMBus
except ImportError:
    from smbus import SMBus


I2C_BUS = 1
HMC5883L_ADDR = 0x1E
QMC5883L_ADDR = 0x0D


def get_i2c_bus_index(preferred):
    if glob.glob(f"/dev/i2c-{preferred}"):
        return preferred
    paths = sorted(glob.glob("/dev/i2c-*"))
    if not paths:
        return None
    try:
        return int(paths[0].split("-")[-1])
    except ValueError:
        return None


def detect_compass(bus):
    for addr in (HMC5883L_ADDR, QMC5883L_ADDR):
        try:
            bus.read_byte(addr)
            return addr
        except OSError:
            continue
    return None


def init_compass(bus, addr):
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


def read_compass(bus, addr):
    if addr == HMC5883L_ADDR:
        data = bus.read_i2c_block_data(addr, 0x03, 6)
        x = (data[0] << 8) | data[1]
        z = (data[2] << 8) | data[3]
        y = (data[4] << 8) | data[5]
        x = x - 65536 if x > 32767 else x
        y = y - 65536 if y > 32767 else y
        z = z - 65536 if z > 32767 else z
    elif addr == QMC5883L_ADDR:
        data = bus.read_i2c_block_data(addr, 0x00, 6)
        x = (data[1] << 8) | data[0]
        y = (data[3] << 8) | data[2]
        z = (data[5] << 8) | data[4]
        x = x - 65536 if x > 32767 else x
        y = y - 65536 if y > 32767 else y
        z = z - 65536 if z > 32767 else z
    else:
        raise ValueError("Unsupported compass address")

    heading = math.degrees(math.atan2(y, x))
    heading = (heading + 360.0) % 360.0
    return heading, (x, y, z)


def nmea_to_decimal(value, hemisphere):
    if not value or not hemisphere:
        return None
    try:
        deg_len = 2 if hemisphere in ("N", "S") else 3
        degrees = float(value[:deg_len])
        minutes = float(value[deg_len:])
        decimal = degrees + (minutes / 60.0)
        if hemisphere in ("S", "W"):
            decimal = -decimal
        return decimal
    except ValueError:
        return None


def parse_nmea_line(line):
    if not line.startswith("$"):
        return None
    parts = line.split(",")
    msg = parts[0]
    if msg in ("$GPGGA", "$GNGGA") and len(parts) >= 6:
        lat = nmea_to_decimal(parts[2], parts[3])
        lon = nmea_to_decimal(parts[4], parts[5])
        fix = parts[6] if len(parts) > 6 else ""
        if lat is None or lon is None:
            return None
        return {"type": "GGA", "lat": lat, "lon": lon, "fix": fix}
    if msg in ("$GPRMC", "$GNRMC") and len(parts) >= 7:
        status = parts[2]
        lat = nmea_to_decimal(parts[3], parts[4])
        lon = nmea_to_decimal(parts[5], parts[6])
        if status != "A" or lat is None or lon is None:
            return None
        return {"type": "RMC", "lat": lat, "lon": lon, "status": status}
    return None


def main():
    gps_ports = ["/dev/ttyAMA4"]
    ser = None
    gps_port = None
    for port in gps_ports:
        try:
            ser = serial.Serial(port, 9600, timeout=0.2)
            gps_port = port
            break
        except (serial.SerialException, FileNotFoundError, PermissionError):
            continue
    if ser is None:
        raise SystemExit("No GPS serial port found. Check wiring/permissions.")
    print(f"Listening to GPS on {gps_port}...\n")

    bus = None
    compass_addr = None
    bus_index = get_i2c_bus_index(I2C_BUS)
    if bus_index is None:
        print("No I2C bus found under /dev/i2c-*; compass disabled.")
    else:
        try:
            bus = SMBus(bus_index)
            print(f"Using I2C bus {bus_index}.")
            compass_addr = detect_compass(bus)
            if compass_addr is None:
                print(f"Compass not found on I2C bus {bus_index}.")
            else:
                init_compass(bus, compass_addr)
                print(f"Compass detected at 0x{compass_addr:02X}\n")
        except FileNotFoundError:
            print(f"/dev/i2c-{bus_index} not found; compass disabled.")

    try:
        last_compass = 0.0
        last_gps = time.time()
        warned_no_gps = False
        while True:
            line = ser.readline()
            if line:
                decoded = line.decode("ascii", errors="ignore").strip()
                gps = parse_nmea_line(decoded)
                if gps:
                    fix_info = gps.get("fix", gps.get("status", ""))
                    print(
                        f"GPS: lat={gps['lat']:.6f} lon={gps['lon']:.6f} "
                        f"src={gps['type']} fix={fix_info}"
                    )
                    last_gps = time.time()
                    warned_no_gps = False
                else:
                    print(f"NMEA: {decoded}")
                    last_gps = time.time()
                    warned_no_gps = False
            elif not warned_no_gps and time.time() - last_gps > 5.0:
                print(
                    f"No GPS data on {gps_port}. Check baud, wiring, and module fix."
                )
                warned_no_gps = True

            now = time.time()
            if bus is not None and compass_addr is not None and now - last_compass >= 0.5:
                last_compass = now
                try:
                    heading, (x, y, z) = read_compass(bus, compass_addr)
                    print(f"Compass: heading={heading:6.2f} deg raw=({x},{y},{z})")
                except OSError:
                    print("Compass read error")
    finally:
        if bus is not None:
            bus.close()


if __name__ == "__main__":
    main()
