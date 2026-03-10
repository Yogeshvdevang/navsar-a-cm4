import cv2
import numpy as np
import time
from pymavlink import mavutil

# ================= MAVLINK =================
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
master.wait_heartbeat()
print("Pixhawk connected")

lidar_height = None
last_valid_height = 1.0  # meters (fallback)

def update_lidar():
    global lidar_height, last_valid_height
    msg = master.recv_match(type='DISTANCE_SENSOR', blocking=False)
    if msg:
        h = msg.current_distance / 100.0
        lidar_height = h
        if 0.2 < h < 10.0:
            last_valid_height = h  # store good height

# ================= CAMERA =================
cap = cv2.VideoCapture(0)

ret, prev_frame = cap.read()
if not ret:
    print("Camera error")
    exit(1)

prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)

# ================= ODOMETRY =================
x, y, z = 0.0, 0.0, 0.0
FOCAL_LENGTH = 350.0  # tuned for 320x240

print("VO + LiDAR FIXED started")

# ================= MAIN LOOP =================
while True:
    update_lidar()

    ret, frame = cap.read()
    if not ret:
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Always re-detect features
    features = cv2.goodFeaturesToTrack(
        prev_gray,
        maxCorners=300,
        qualityLevel=0.01,
        minDistance=7
    )

    if features is None:
        prev_gray = gray
        continue

    next_pts, status, _ = cv2.calcOpticalFlowPyrLK(
        prev_gray, gray, features, None,
        winSize=(15, 15), maxLevel=2
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
    height = last_valid_height
    z = height  # ALWAYS update Z

    dx_m = (dx_pixels / FOCAL_LENGTH) * height
    dy_m = (dy_pixels / FOCAL_LENGTH) * height

    x += dx_m
    y += dy_m

    # ---------------- DIRECTION ----------------
    if abs(dx_m) > abs(dy_m):
        direction = "FORWARD" if dx_m > 0 else "BACKWARD"
    else:
        direction = "RIGHT" if dy_m > 0 else "LEFT"

    # ---------------- VISUAL DEBUG ----------------
    for p in good_new[:50]:
        cv2.circle(frame, (int(p[0]), int(p[1])), 2, (0, 0, 255), -1)

    center = (160, 120)
    arrow_end = (
        int(center[0] + dy_pixels * 8),
        int(center[1] + dx_pixels * 8)
    )
    cv2.arrowedLine(frame, center, arrow_end, (0, 255, 0), 2)

    # ---------------- OVERLAY ----------------
    cv2.putText(frame, f"DIR: {direction}", (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    cv2.putText(frame,
                f"dx_pix: {dx_pixels:.2f} dy_pix: {dy_pixels:.2f}",
                (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    cv2.putText(frame,
                f"X: {x:.2f} Y: {y:.2f} Z: {z:.2f}",
                (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    cv2.putText(frame,
                f"dX: {dx_m:.3f} dY: {dy_m:.3f}",
                (10, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    cv2.imshow("VO + LiDAR FIXED", frame)

    if cv2.waitKey(1) & 0xFF == 27:
        break

    prev_gray = gray
    time.sleep(0.02)

cap.release()
cv2.destroyAllWindows()
