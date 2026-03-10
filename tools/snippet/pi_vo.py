import time
import cv2
import numpy as np
from picamera2 import Picamera2

# ---------------- CONFIG ----------------
FRAME_SIZE = (640, 400)
FOCAL_LENGTH_PX = 350.0  # tune for your lens/resolution
HEIGHT_M = 1.0  # set to actual height if camera is nadir; used for scale
MAX_CORNERS = 300
QUALITY_LEVEL = 0.01
MIN_DISTANCE = 7
FB_ERR_THRESH = 1.0
MIN_FEATURES = 30

# Wide-angle support: set these from camera calibration.
# K is camera matrix, D is fisheye distortion (k1,k2,k3,k4).
USE_UNDISTORT = True
K = np.array(
    [[350.0, 0.0, FRAME_SIZE[0] / 2.0],
     [0.0, 350.0, FRAME_SIZE[1] / 2.0],
     [0.0, 0.0, 1.0]],
    dtype=np.float32,
)
D = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32)

# Initialize the camera
picam2 = Picamera2()
config = picam2.create_video_configuration(main={"format": "YUV420", "size": FRAME_SIZE})
picam2.configure(config)
picam2.start()

print("OV9281 VO (Median Flow) Started. Press 'q' to quit.")

# ---------------- STATE ----------------
x, y, z = 0.0, 0.0, HEIGHT_M
prev_gray = None
prev_pts = None
prev_time = None
map1 = None
map2 = None


def detect_features(gray):
    return cv2.goodFeaturesToTrack(
        gray,
        maxCorners=MAX_CORNERS,
        qualityLevel=QUALITY_LEVEL,
        minDistance=MIN_DISTANCE,
    )


def median_flow(prev_gray, gray, prev_pts):
    next_pts, st, _ = cv2.calcOpticalFlowPyrLK(
        prev_gray, gray, prev_pts, None, winSize=(21, 21), maxLevel=3
    )
    if next_pts is None:
        return None, None, None

    # Forward-backward check for robust median flow
    back_pts, st_back, _ = cv2.calcOpticalFlowPyrLK(
        gray, prev_gray, next_pts, None, winSize=(21, 21), maxLevel=3
    )
    if back_pts is None:
        return None, None, None

    prev_good = prev_pts[st == 1]
    next_good = next_pts[st == 1]
    back_good = back_pts[st == 1]

    fb_err = np.linalg.norm(prev_good - back_good, axis=1)
    keep = fb_err < FB_ERR_THRESH
    prev_good = prev_good[keep]
    next_good = next_good[keep]

    if len(prev_good) < MIN_FEATURES:
        return None, None, None

    flow = next_good - prev_good
    dx = np.median(flow[:, 0])
    dy = np.median(flow[:, 1])
    return dx, dy, next_good


try:
    while True:
        frame = picam2.capture_array()
        gray = frame[:FRAME_SIZE[1], :FRAME_SIZE[0]]
        if USE_UNDISTORT:
            if map1 is None or map2 is None:
                new_k = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                    K, D, FRAME_SIZE, np.eye(3), balance=0.0
                )
                map1, map2 = cv2.fisheye.initUndistortRectifyMap(
                    K, D, np.eye(3), new_k, FRAME_SIZE, cv2.CV_16SC2
                )
            gray = cv2.remap(gray, map1, map2, interpolation=cv2.INTER_LINEAR)

        now = time.time()
        if prev_gray is None:
            prev_gray = gray
            prev_pts = detect_features(prev_gray)
            prev_time = now
            continue

        if prev_pts is None or len(prev_pts) < MIN_FEATURES:
            prev_pts = detect_features(prev_gray)
            prev_time = now
            continue

        dx_px, dy_px, next_pts = median_flow(prev_gray, gray, prev_pts)
        if dx_px is None:
            prev_gray = gray
            prev_pts = detect_features(prev_gray)
            prev_time = now
            continue

        dt = max(now - prev_time, 1e-6)

        dx_m = (dx_px / FOCAL_LENGTH_PX) * HEIGHT_M
        dy_m = (dy_px / FOCAL_LENGTH_PX) * HEIGHT_M
        dz_m = 0.0

        x += dx_m
        y += dy_m
        z = HEIGHT_M

        vx = dx_m / dt
        vy = dy_m / dt
        vz = dz_m / dt

        # ---------------- VISUAL DEBUG ----------------
        vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        for p in next_pts[:50]:
            cv2.circle(vis, (int(p[0]), int(p[1])), 2, (0, 0, 255), -1)

        center = (FRAME_SIZE[0] // 2, FRAME_SIZE[1] // 2)
        arrow_end = (
            int(center[0] + dy_px * 8),
            int(center[1] + dx_px * 8),
        )
        cv2.arrowedLine(vis, center, arrow_end, (0, 255, 0), 2)

        cv2.putText(vis, f"x={x:.3f} y={y:.3f} z={z:.3f}", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(vis, f"dx={dx_m:.4f} dy={dy_m:.4f} dz={dz_m:.4f}", (10, 45),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        cv2.putText(vis, f"vx={vx:.3f} vy={vy:.3f} vz={vz:.3f}", (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        cv2.imshow("OV9281 Odometry Feed", vis)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        prev_gray = gray
        prev_pts = next_pts.reshape(-1, 1, 2)
        prev_time = now
finally:
    picam2.stop()
    cv2.destroyAllWindows()
