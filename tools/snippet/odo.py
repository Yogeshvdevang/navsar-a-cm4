import cv2
import numpy as np
import time
from pymavlink import mavutil

# ---------------- MAVLINK ----------------
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
master.wait_heartbeat()
print("Pixhawk connected")

lidar_height = None
last_valid_height = 1.0  # fallback height in meters

def update_lidar():
    global lidar_height, last_valid_height
    msg = master.recv_match(type='DISTANCE_SENSOR', blocking=False)
    if msg:
        h = msg.current_distance / 100.0
        lidar_height = h
        if 0.2 < h < 10.0:
            last_valid_height = h

# ---------------- CAMERA ----------------
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

ret, prev_frame = cap.read()
if not ret:
    print("Camera error")
    exit(1)

prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)

# ---------------- ODOMETRY ----------------
x, y, z = 0.0, 0.0, 0.0
FOCAL_LENGTH = 600.0

print("Visual odometry started")

# ---------------- MAIN LOOP ----------------
while True:
    update_lidar()

    ret, frame = cap.read()
    if not ret:
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # ALWAYS re-detect features
    features = cv2.goodFeaturesToTrack(
        prev_gray,
        maxCorners=300,
        qualityLevel=0.01,
        minDistance=7
    )

    if features is None:
        prev_gray = gray
        continue

    next_pts, status, err = cv2.calcOpticalFlowPyrLK(
        prev_gray, gray, features, None,
        winSize=(21, 21), maxLevel=3
    )

    if next_pts is None:
        prev_gray = gray
        continue

    good_old = features[status == 1]
    good_new = next_pts[status == 1]

    if len(good_old) < 10:
        prev_gray = gray
        continue

    # ---------------- PIXEL MOTION ----------------
    dx_pixels = np.mean(good_new[:, 0] - good_old[:, 0])
    dy_pixels = np.mean(good_new[:, 1] - good_old[:, 1])

    # ---------------- METRIC MOTION ----------------
    z = last_valid_height
    dx_m = (dx_pixels / FOCAL_LENGTH) * z
    dy_m = (dy_pixels / FOCAL_LENGTH) * z

    x += dx_m
    y += dy_m

    print(
        f"X: {x:.3f} m | Y: {y:.3f} m | Z: {z:.2f} m | "
        f"dX: {dx_m:.4f} | dY: {dy_m:.4f} | "
        f"dx_pix: {dx_pixels:.2f} dy_pix: {dy_pixels:.2f}"
    )

    prev_gray = gray
    time.sleep(0.02)
