

import time
import cv2
import numpy as np

# ================= CONFIG =================
CAMERA_INDEX = 1
IMG_WIDTH = 640
IMG_HEIGHT = 480

# Camera intrinsics (replace with calibrated values)
FX = 525.0
FY = 525.0
CX = IMG_WIDTH / 2.0
CY = IMG_HEIGHT / 2.0
K = np.array([[FX, 0.0, CX],
              [0.0, FY, CY],
              [0.0, 0.0, 1.0]], dtype=np.float64)
DIST_COEFFS = None


# Optical flow (Lucas-Kanade)
MIN_FEATURES = 40
MAX_FEATURES = 300
REDETECT_INTERVAL = 10

# Scale source
USE_LIDAR = False
ALTITUDE_M = 1.0
DRONE_DATA = None

# MAVLink I/O
MAVLINK_DEVICE = "/dev/ttyACM0"
MAVLINK_BAUD = 115200
FRAME_ID = 1          # MAV_FRAME_LOCAL_NED (common for odom)
CHILD_FRAME_ID = 8    # MAV_FRAME_BODY_FRD
ESTIMATOR_TYPE = 2    # MAV_ESTIMATOR_TYPE_VISION

# ================= MAVLINK =================
# Pixhawk is currently disconnected; keep MAVLink/LiDAR disabled.
# master = mavutil.mavlink_connection(MAVLINK_DEVICE, baud=MAVLINK_BAUD)
# master.wait_heartbeat()
# print("Pixhawk connected")

lidar_height = None
last_valid_height = ALTITUDE_M


# def update_lidar():
#     global lidar_height, last_valid_height
#     msg = master.recv_match(type="DISTANCE_SENSOR", blocking=False)
#     if msg:
#         h = msg.current_distance / 100.0
#         lidar_height = h
#         if 0.2 < h < 10.0:
#             last_valid_height = h


def format_odometry(time_usec, frame_id, child_frame_id, pos, quat, vel,
                    rates, pose_cov, vel_cov, reset_counter, estimator_type):
    return (
        "ODOMETRY(\n"
        f"  {time_usec},\n"
        f"  {frame_id},\n"
        f"  {child_frame_id},\n"
        f"  {pos[0]:.6f}, {pos[1]:.6f}, {pos[2]:.6f},\n"
        f"  [{quat[0]:.6f}, {quat[1]:.6f}, {quat[2]:.6f}, {quat[3]:.6f}],\n"
        f"  {vel[0]:.6f}, {vel[1]:.6f}, {vel[2]:.6f},\n"
        f"  {rates[0]:.6f}, {rates[1]:.6f}, {rates[2]:.6f},\n"
        f"  {pose_cov},\n"
        f"  {vel_cov},\n"
        f"  {reset_counter},\n"
        f"  {estimator_type}\n"
        ")"
    )


# ================= CAMERA =================
cap = cv2.VideoCapture(CAMERA_INDEX)
if IMG_WIDTH and IMG_HEIGHT:
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMG_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_HEIGHT)


ret, prev_frame = cap.read()
if not ret:
    print("Camera error")
    raise SystemExit(1)

prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)
if DIST_COEFFS is not None:
    prev_gray = cv2.undistort(prev_gray, K, DIST_COEFFS)

# Tracking params
lk_params = dict(
    winSize=(15, 15),
    maxLevel=2,
    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
)
feature_params = dict(
    maxCorners=MAX_FEATURES,
    qualityLevel=0.2,
    minDistance=7,
    blockSize=7,
)


# Odom state
x, y, z = 0.0, 0.0, 0.0
p0 = cv2.goodFeaturesToTrack(prev_gray, mask=None, **feature_params)
frame_idx = 0
reset_counter = 0
last_time = time.time()

print("VIO started")

# ================= MAIN LOOP =================
while True:
    # update_lidar()

    ret, frame = cap.read()
    if not ret:
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if DIST_COEFFS is not None:
        gray = cv2.undistort(gray, K, DIST_COEFFS)

    if p0 is None or len(p0) < MIN_FEATURES or (frame_idx % REDETECT_INTERVAL == 0):
        p0 = cv2.goodFeaturesToTrack(prev_gray, mask=None, **feature_params)
        if p0 is None:
            prev_gray = gray
            frame_idx += 1
            continue
    p1, st, _ = cv2.calcOpticalFlowPyrLK(prev_gray, gray, p0, None, **lk_params)
    if p1 is None:
        prev_gray = gray
        frame_idx += 1
        continue

    good_old = p0[st == 1]
    good_new = p1[st == 1]
    if len(good_old) < 8:
        prev_gray = gray
        frame_idx += 1
        continue

    if len(good_old) < 8:
        prev_gray = gray
        frame_idx += 1
        continue

    dx_pixels = np.median(good_new[:, 0] - good_old[:, 0])
    dy_pixels = np.median(good_new[:, 1] - good_old[:, 1])

    height = last_valid_height if USE_LIDAR else ALTITUDE_M
    z = height

    dx_m = (dx_pixels / FX) * height
    dy_m = (dy_pixels / FY) * height
    dz_m = 0.0

    x += dx_m
    y += dy_m

    now = time.time()
    dt = max(1e-3, now - last_time)
    vx = dx_m / dt
    vy = dy_m / dt
    vz = dz_m / dt
    last_time = now

    time_usec = int(now * 1e6)
    quat = [1.0, 0.0, 0.0, 0.0]  # unknown orientation -> identity
    rates = [0.0, 0.0, 0.0]
    pose_cov = [0.0] * 21
    vel_cov = [0.0] * 21

    odom_text = format_odometry(
        time_usec=time_usec,
        frame_id=FRAME_ID,
        child_frame_id=CHILD_FRAME_ID,
        pos=(x, y, z),
        quat=quat,
        vel=(vx, vy, vz),
        rates=rates,
        pose_cov=pose_cov,
        vel_cov=vel_cov,
        reset_counter=reset_counter,
        estimator_type=ESTIMATOR_TYPE,
    )
    print(odom_text)
    print(f"drone_data={DRONE_DATA}")

    overlay_lines = [
        "VPS -> Pixhawk ODOMETRY fields",
        f"time_usec: {time_usec}",
        f"frame_id: {FRAME_ID} child_frame_id: {CHILD_FRAME_ID}",
        f"pos[m]: x={x:.2f} y={y:.2f} z={z:.2f}",
        f"vel[m/s]: vx={vx:.2f} vy={vy:.2f} vz={vz:.2f}",
        f"quat[wxyz]: {quat[0]:.2f} {quat[1]:.2f} {quat[2]:.2f} {quat[3]:.2f}",
        f"rates[rad/s]: {rates[0]:.2f} {rates[1]:.2f} {rates[2]:.2f}",
        f"reset_counter: {reset_counter} estimator_type: {ESTIMATOR_TYPE}",
        f"features: {len(good_new)}",
        f"scale_source: {'lidar' if USE_LIDAR else 'fixed'} height={height:.2f}m",
        f"drone_data: {DRONE_DATA}",
    ]
    y0 = 20
    for i, line in enumerate(overlay_lines):
        y_pos = y0 + i * 18
        cv2.putText(
            frame,
            line,
            (10, y_pos),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )

    cv2.imshow("VIO Camera", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

    prev_gray = gray
    p0 = good_new.reshape(-1, 1, 2)
    frame_idx += 1
    time.sleep(0.02)

cap.release()
cv2.destroyAllWindows()
