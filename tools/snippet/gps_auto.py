import serial
import time
import math
import struct
from datetime import datetime

# ================== CONFIG ==================
SERIAL_PORT = "/dev/ttyAMA0"  # "/dev/ttyUSB0"   #"/dev/ttyAMA2"    #"COM30"   # change if needed
BAUDRATE = 230400
UPDATE_RATE_HZ = 5      # 5-10 Hz works well

# Start location
lat = 12.971600         # degrees
lon = 77.594600         # degrees
alt = 920.0             # meters

speed_mps = 1.5         # fake ground speed
heading_deg = 90.0      # east
num_sats = 10
# ============================================


def nmea_checksum(sentence):
    cs = 0
    for c in sentence:
        cs ^= ord(c)
    return f"{cs:02X}"


def ubx_checksum(msg_class, msg_id, payload):
    """Calculate UBX checksum (Fletcher algorithm)"""
    ck_a = 0
    ck_b = 0
    
    ck_a = (ck_a + msg_class) & 0xFF
    ck_b = (ck_b + ck_a) & 0xFF
    
    ck_a = (ck_a + msg_id) & 0xFF
    ck_b = (ck_b + ck_a) & 0xFF
    
    length = len(payload)
    ck_a = (ck_a + (length & 0xFF)) & 0xFF
    ck_b = (ck_b + ck_a) & 0xFF
    
    ck_a = (ck_a + ((length >> 8) & 0xFF)) & 0xFF
    ck_b = (ck_b + ck_a) & 0xFF
    
    for byte in payload:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    
    return ck_a, ck_b


def create_ubx_message(msg_class, msg_id, payload):
    """Create a complete UBX message with header and checksum"""
    length = len(payload)
    header = struct.pack('<BBBB', 0xB5, 0x62, msg_class, msg_id)
    length_bytes = struct.pack('<H', length)
    ck_a, ck_b = ubx_checksum(msg_class, msg_id, payload)
    checksum = struct.pack('BB', ck_a, ck_b)
    
    return header + length_bytes + payload + checksum


def create_ubx_nav_posllh(lat_deg, lon_deg, alt_m, time_of_week_ms):
    """UBX-NAV-POSLLH: Position solution in LLH format"""
    lat_1e7 = int(lat_deg * 1e7)
    lon_1e7 = int(lon_deg * 1e7)
    alt_mm = int(alt_m * 1000)
    h_msl_mm = int(alt_m * 1000)
    h_acc_mm = 2000  # horizontal accuracy estimate
    v_acc_mm = 3000  # vertical accuracy estimate
    
    payload = struct.pack('<IiiiiII',
                         time_of_week_ms,
                         lon_1e7,
                         lat_1e7,
                         alt_mm,
                         h_msl_mm,
                         h_acc_mm,
                         v_acc_mm)
    
    return create_ubx_message(0x01, 0x02, payload)


def create_ubx_nav_velned(speed_mps, heading_deg, time_of_week_ms):
    """UBX-NAV-VELNED: Velocity solution in NED frame"""
    heading_rad = math.radians(heading_deg)
    vel_n_cm = int(speed_mps * 100 * math.cos(heading_rad))
    vel_e_cm = int(speed_mps * 100 * math.sin(heading_rad))
    vel_d_cm = 0  # no vertical velocity
    speed_cm = int(speed_mps * 100)
    ground_speed_cm = speed_cm
    heading_1e5 = int(heading_deg * 1e5)
    s_acc_cm = 50   # speed accuracy
    c_acc_1e5 = 5000  # course accuracy
    
    payload = struct.pack('<IiiiIIiII',
                         time_of_week_ms,
                         vel_n_cm,
                         vel_e_cm,
                         vel_d_cm,
                         speed_cm,
                         ground_speed_cm,
                         heading_1e5,
                         s_acc_cm,
                         c_acc_1e5)
    
    return create_ubx_message(0x01, 0x12, payload)


def create_ubx_nav_sol(num_sats, time_of_week_ms):
    """UBX-NAV-SOL: Navigation solution"""
    gps_fix = 3  # 3D fix
    flags = 0x07  # gnssFixOK | diffSoln | WKNSET | TOWSET
    p_acc_cm = 250  # position accuracy
    
    payload = struct.pack('<IihBBIiiiIIHBBII',
                         time_of_week_ms,
                         0,  # fTOW (fractional time)
                         0,  # week
                         gps_fix,
                         flags,
                         0, 0, 0,  # ecefX, Y, Z (not used)
                         p_acc_cm,
                         0, 0,  # ecefVX, VY, VZ
                         50,  # sAcc
                         0,  # pDOP (scaled)
                         0,  # reserved1
                         num_sats,
                         0)  # reserved2
    
    return create_ubx_message(0x01, 0x06, payload)


def create_ubx_nav_pvt(lat_deg, lon_deg, alt_m, speed_mps, heading_deg, num_sats, now):
    """UBX-NAV-PVT: Navigation position velocity time solution (preferred message)"""
    gps_fix = 3  # 3D fix
    flags = 0x01  # gnssFixOK
    flags2 = 0x00
    
    heading_rad = math.radians(heading_deg)
    vel_n_mm = int(speed_mps * 1000 * math.cos(heading_rad))
    vel_e_mm = int(speed_mps * 1000 * math.sin(heading_rad))
    vel_d_mm = 0
    
    lat_1e7 = int(lat_deg * 1e7)
    lon_1e7 = int(lon_deg * 1e7)
    alt_mm = int(alt_m * 1000)
    h_msl_mm = int(alt_m * 1000)
    
    # GPS time (simplified - using system time)
    time_of_week_ms = (now.hour * 3600 + now.minute * 60 + now.second) * 1000 + now.microsecond // 1000
    
    payload = struct.pack('<IHBBBBBBIiBBBBiiiiIIiiiiiIIHBBBBBBi',
                         time_of_week_ms,
                         now.year,
                         now.month,
                         now.day,
                         now.hour,
                         now.minute,
                         now.second,
                         flags,
                         0,  # tAcc
                         0,  # nano
                         gps_fix,
                         flags,
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
                         int(speed_mps * 1000),  # gSpeed
                         int(heading_deg * 1e5),  # headMot
                         1000,  # sAcc
                         10000,  # headAcc
                         0,  # pDOP
                         0, 0, 0, 0, 0, 0,  # reserved and flags3
                         0)  # headVeh
    
    return create_ubx_message(0x01, 0x07, payload)


def deg_to_nmea(deg, is_lat):
    d = int(abs(deg))
    m = (abs(deg) - d) * 60
    if is_lat:
        return f"{d:02d}{m:07.4f}", "N" if deg >= 0 else "S"
    return f"{d:03d}{m:07.4f}", "E" if deg >= 0 else "W"


def nmea_time(now):
    centisec = int(now.microsecond / 10000)
    return now.strftime("%H%M%S") + f".{centisec:02d}"


ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
dt = 1.0 / UPDATE_RATE_HZ

print("Fake GPS (UBX + NMEA) streaming to Pixhawk...")
print("Compatible with GPS_TYPE = 1 (AUTO) or 5 (NMEA)\n")

while True:

    # Add small realistic variations
    alt += math.sin(time.time() * 0.5) * 0.2  # �0.2m altitude drift
    heading_deg += math.sin(time.time() * 0.3) * 2  # �2� heading variation
    speed_mps += math.sin(time.time() * 0.7) * 0.1  # slight speed variation
    # --- Move position ---
    heading_rad = math.radians(heading_deg)
    dx = speed_mps * dt * math.cos(heading_rad)
    dy = speed_mps * dt * math.sin(heading_rad)

    lat += dy / 111111.0
    lon += dx / (111111.0 * math.cos(math.radians(lat)))

    now = datetime.utcnow()
    time_of_week_ms = (now.hour * 3600 + now.minute * 60 + now.second) * 1000 + now.microsecond // 1000
    
    # ========== SEND UBX MESSAGES (for AUTO mode) ==========
    ubx_pvt = create_ubx_nav_pvt(lat, lon, alt, speed_mps, heading_deg, num_sats, now)
    ubx_posllh = create_ubx_nav_posllh(lat, lon, alt, time_of_week_ms)
    ubx_velned = create_ubx_nav_velned(speed_mps, heading_deg, time_of_week_ms)
    ubx_sol = create_ubx_nav_sol(num_sats, time_of_week_ms)
    
    ser.write(ubx_pvt)
    ser.write(ubx_posllh)
    ser.write(ubx_velned)
    ser.write(ubx_sol)
    
    # ========== SEND NMEA MESSAGES (for fallback/compatibility) ==========
    t_str = nmea_time(now)
    d_str = now.strftime("%d%m%y")

    lat_nmea, lat_dir = deg_to_nmea(lat, True)
    lon_nmea, lon_dir = deg_to_nmea(lon, False)

    # GGA
    gga_body = (
        f"GPGGA,{t_str},{lat_nmea},{lat_dir},"
        f"{lon_nmea},{lon_dir},1,{num_sats:02d},0.8,"
        f"{alt:.1f},M,-34.0,M,,"
    )
    gga = f"${gga_body}*{nmea_checksum(gga_body)}"

    # RMC
    speed_knots = speed_mps * 1.94384
    rmc_body = (
        f"GPRMC,{t_str},A,{lat_nmea},{lat_dir},"
        f"{lon_nmea},{lon_dir},{speed_knots:.2f},"
        f"{heading_deg:.1f},{d_str},,,A"
    )
    rmc = f"${rmc_body}*{nmea_checksum(rmc_body)}"

    ser.write((gga + "\r\n").encode())
    ser.write((rmc + "\r\n").encode())

    # ---- PRINT STATUS ----
    print(f"LAT : {lat:.6f}� {'N' if lat >= 0 else 'S'}")
    print(f"LON : {lon:.6f}� {'E' if lon >= 0 else 'W'}")
    print(f"ALT : {alt:.1f} m")
    print(f"SPD : {speed_mps:.2f} m/s")
    print(f"HDG : {heading_deg:.1f}�")
    print(f"SATS: {num_sats}")
    print(f"Sent: UBX-NAV-PVT, POSLLH, VELNED, SOL + NMEA GGA, RMC")
    print("-" * 50)

    time.sleep(dt)

    """
    right now do this in optical flow gps port mode 
take the origin from gps sensor and  vx and vy from the optical flow sensor and from these data calculate the realtime latitude and longitude nothing fancy in this just normal optical flow to gps data into lat longitude 
    """