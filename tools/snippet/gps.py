import serial

SERIAL_PORT = "/dev/ttyAMA5"
BAUDRATE = 9600


def nmea_to_decimal(raw, direction):
    if not raw or not direction:
        return None
    try:
        deg_len = 2 if direction in ("N", "S") else 3
        deg = float(raw[:deg_len])
        minutes = float(raw[deg_len:])
        dec = deg + minutes / 60.0
        if direction in ("S", "W"):
            dec = -dec
        return dec
    except ValueError:
        return None


def parse_gga(sentence):
    if not sentence.startswith(("$GPGGA", "$GNGGA")):
        return None
    body = sentence.split("*", 1)[0]
    parts = body.split(",")
    if len(parts) < 10:
        return None
    lat_raw, lat_dir = parts[2], parts[3]
    lon_raw, lon_dir = parts[4], parts[5]
    alt_raw = parts[9]
    lat = nmea_to_decimal(lat_raw, lat_dir)
    lon = nmea_to_decimal(lon_raw, lon_dir)
    try:
        alt = float(alt_raw)
    except ValueError:
        alt = None
    if lat is None or lon is None or alt is None:
        return None
    return lat, lon, alt


def parse_rmc(sentence):
    if not sentence.startswith(("$GPRMC", "$GNRMC")):
        return None
    body = sentence.split("*", 1)[0]
    parts = body.split(",")
    if len(parts) < 7:
        return None
    lat_raw, lat_dir = parts[3], parts[4]
    lon_raw, lon_dir = parts[5], parts[6]
    lat = nmea_to_decimal(lat_raw, lat_dir)
    lon = nmea_to_decimal(lon_raw, lon_dir)
    if lat is None or lon is None:
        return None
    return lat, lon


ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
print("Listening to GPS...\n")

while True:
    line = ser.readline()
    if not line:
        continue
    sentence = line.decode("ascii", errors="ignore").strip()
    if not sentence:
        continue
    print(f"RAW: {sentence}")
    parsed_gga = parse_gga(sentence)
    if parsed_gga:
        lat, lon, alt = parsed_gga
        print(f"GGA: LAT {lat:.6f}  LON {lon:.6f}  ALT {alt:.1f} m")
        continue
    parsed_rmc = parse_rmc(sentence)
    if parsed_rmc:
        lat, lon = parsed_rmc
        print(f"RMC: LAT {lat:.6f}  LON {lon:.6f}")
