import serial
import time
import math
from datetime import datetime

# ================== CONFIG ==================
SERIAL_PORT = "/dev/ttyUSB0"   # change if needed
BAUDRATE = 9600
UPDATE_RATE_HZ = 5             # 5�10 Hz

# Start location
lat = 12.971600        # degrees
lon = 77.594600        # degrees
alt = 920.0            # meters

speed_mps = 1.5        # fake ground speed
heading_deg = 90.0     # east
# ============================================

def nmea_checksum(sentence):
    cs = 0
    for c in sentence:
        cs ^= ord(c)
    return f"{cs:02X}"

def deg_to_nmea(deg, is_lat):
    d = int(abs(deg))
    m = (abs(deg) - d) * 60
    if is_lat:
        return f"{d:02d}{m:07.4f}", "N" if deg >= 0 else "S"
    else:
        return f"{d:03d}{m:07.4f}", "E" if deg >= 0 else "W"

ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
dt = 1.0 / UPDATE_RATE_HZ

print("?? Fake GPS streaming to Pixhawk...\n")

while True:
    # --- Move position ---
    heading_rad = math.radians(heading_deg)
    dx = speed_mps * dt * math.cos(heading_rad)
    dy = speed_mps * dt * math.sin(heading_rad)

    lat += dx / 111111.0
    lon += dy / (111111.0 * math.cos(math.radians(lat)))

    now = datetime.utcnow()
    t_str = now.strftime("%H%M%S")
    d_str = now.strftime("%d%m%y")

    lat_nmea, lat_dir = deg_to_nmea(lat, True)
    lon_nmea, lon_dir = deg_to_nmea(lon, False)

    # -------- GGA --------
    gga_body = (
        f"GPGGA,{t_str},{lat_nmea},{lat_dir},"
        f"{lon_nmea},{lon_dir},1,10,0.8,"
        f"{alt:.1f},M,-34.0,M,,"
    )
    gga = f"${gga_body}*{nmea_checksum(gga_body)}"

    # -------- RMC --------
    speed_knots = speed_mps * 1.94384
    rmc_body = (
        f"GPRMC,{t_str},A,{lat_nmea},{lat_dir},"
        f"{lon_nmea},{lon_dir},{speed_knots:.2f},"
        f"{heading_deg:.1f},{d_str},,,A"
    )
    rmc = f"${rmc_body}*{nmea_checksum(rmc_body)}"

    # ---- SEND TO PIXHAWK ----
    ser.write((gga + "\r\n").encode())
    ser.write((rmc + "\r\n").encode())

    # ---- PRINT RAW NMEA (EXACT DATA) ----
    print(gga)
    print(rmc)

    # ---- PRINT DECODED (HUMAN READABLE) ----
    print(f"LAT : {lat:.6f}� {'N' if lat >= 0 else 'S'}")
    print(f"LON : {lon:.6f}� {'E' if lon >= 0 else 'W'}")
    print(f"ALT : {alt:.1f} m")
    print(f"SPD : {speed_mps:.2f} m/s")
    print(f"HDG : {heading_deg:.1f}�")
    print(f"SATS: 10")
    print("-" * 45)

    time.sleep(dt)
