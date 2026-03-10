import time
import cv2
import numpy as np
from picamera2 import Picamera2

# ---------------- CONFIG ----------------
FRAME_SIZE = (640, 400)  # (width, height)
FPS = 60

# Camera intrinsics (replace with calibrated values)
FX = 350.0
FY = 350.0
CX = FRAME_SIZE[0] / 2.0
CY = FRAME_SIZE[1] / 2.0
K = np.array([[FX, 0.0, CX],
              [0.0, FY, CY],
              [0.0, 0.0, 1.0]], dtype=np.float64)

# Distortion (set to calibrated values if using a wide lens)
USE_UNDISTORT = False
DIST_COEFFS = np.zeros((4, 1), dtype=np.float64)  # k1, k2, p1, p2

# Feature tracking
MAX_FEATURES = 400
MIN_FEATURES = 80
QUALITY_LEVEL = 0.01
MIN_DISTANCE = 7
REDETECT_INTERVAL = 10
FB_ERR_THRESH = 1.0
MIN_FLOW_PX = 0.5
MIN_INLIER_RATIO = 0.3

# Scale (pure monocular VO has unknown scale)
SCALE = 1.0

# ---------------- CAMERA ----------------
picam2 = Picamera2()
config = picam2.create_video_configuration(
    main={"format": "YUV420", "size": FRAME_SIZE},
    controls={"FrameRate": FPS},
)
picam2.configure(config)
picam2.start()

print("RPi5 SVO-style VO started. Press 'q' to quit.")


def detect_features(gray):
    return cv2.goodFeaturesToTrack(
        gray,
        maxCorners=MAX_FEATURES,
        qualityLevel=QUALITY_LEVEL,
        minDistance=MIN_DISTANCE,
    )


def track_features(prev_gray, gray, prev_pts):
    next_pts, st, _ = cv2.calcOpticalFlowPyrLK(
        prev_gray,
        gray,
        prev_pts,
        None,
        winSize=(21, 21),
        maxLevel=3,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
    )
    if next_pts is None:
        return None, None

    back_pts, st_back, _ = cv2.calcOpticalFlowPyrLK(
        gray,
        prev_gray,
        next_pts,
        None,
        winSize=(21, 21),
        maxLevel=3,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
    )
    if back_pts is None:
        return None, None

    good = (st == 1) & (st_back == 1)
    prev_good = prev_pts[good].reshape(-1, 2)
    next_good = next_pts[good].reshape(-1, 2)
    back_good = back_pts[good].reshape(-1, 2)
    if len(prev_good) < MIN_FEATURES:
        return None, None

    fb_err = np.linalg.norm(prev_good - back_good, axis=1)
    keep = fb_err < FB_ERR_THRESH
    prev_good = prev_good[keep]
    next_good = next_good[keep]

    if len(prev_good) < MIN_FEATURES:
        return None, None

    prev_good = prev_good.reshape(-1, 1, 2).astype(np.float32)
    next_good = next_good.reshape(-1, 1, 2).astype(np.float32)
    return prev_good, next_good


def undistort_gray(gray):
    if not USE_UNDISTORT:
        return gray
    return cv2.undistort(gray, K, DIST_COEFFS)


# ---------------- STATE ----------------
prev_gray = None
prev_pts = None
frame_idx = 0
last_time = time.time()

R_world = np.eye(3, dtype=np.float64)
t_world = np.zeros((3, 1), dtype=np.float64)

try:
    while True:
        frame = picam2.capture_array()
        gray = frame[:FRAME_SIZE[1], :FRAME_SIZE[0]]
        gray = undistort_gray(gray)

        if prev_gray is None:
            prev_gray = gray
            prev_pts = detect_features(prev_gray)
            last_time = time.time()
            continue

        if prev_pts is None or len(prev_pts) < MIN_FEATURES or frame_idx % REDETECT_INTERVAL == 0:
            prev_pts = detect_features(prev_gray)
            if prev_pts is None:
                prev_gray = gray
                frame_idx += 1
                continue

        tracked = track_features(prev_gray, gray, prev_pts)
        if tracked is None or tracked[0] is None:
            prev_gray = gray
            prev_pts = detect_features(prev_gray)
            frame_idx += 1
            continue
        prev_good, next_good = tracked

        if len(prev_good) < 8 or len(next_good) < 8:
            prev_gray = gray
            prev_pts = next_good
            frame_idx += 1
            continue

        flow = next_good.reshape(-1, 2) - prev_good.reshape(-1, 2)
        flow_med = np.median(np.linalg.norm(flow, axis=1))
        if flow_med < MIN_FLOW_PX:
            prev_gray = gray
            prev_pts = next_good
            frame_idx += 1
            continue

        E, mask = cv2.findEssentialMat(
            prev_good,
            next_good,
            K,
            method=cv2.RANSAC,
            prob=0.999,
            threshold=1.0,
        )
        if E is None:
            prev_gray = gray
            prev_pts = next_good
            frame_idx += 1
            continue

        _, R, t, mask_pose = cv2.recoverPose(E, prev_good, next_good, K)
        inliers = int(mask_pose.sum()) if mask_pose is not None else 0
        if inliers < MIN_INLIER_RATIO * len(prev_good):
            prev_gray = gray
            prev_pts = next_good
            frame_idx += 1
            continue

        delta_t = R_world @ (t * SCALE)
        t_world = t_world + delta_t
        R_world = R_world @ R

        now = time.time()
        dt = max(1e-3, now - last_time)
        last_time = now

        vx, vy, vz = (delta_t[0, 0] / dt, delta_t[1, 0] / dt, delta_t[2, 0] / dt)

        vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        for p in next_good[:50]:
            cv2.circle(vis, (int(p[0, 0]), int(p[0, 1])), 2, (0, 0, 255), -1)

        cv2.putText(
            vis,
            f"x={t_world[0, 0]:.3f} y={t_world[1, 0]:.3f} z={t_world[2, 0]:.3f}",
            (10, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
        )
        cv2.putText(
            vis,
            f"vx={vx:.3f} vy={vy:.3f} vz={vz:.3f}",
            (10, 45),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),
            1,
        )
        cv2.putText(
            vis,
            f"features={len(next_good)}",
            (10, 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),
            1,
        )
        cv2.putText(
            vis,
            f"flow_med={flow_med:.2f}px inliers={inliers}",
            (10, 95),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),
            1,
        )

        cv2.imshow("RPi5 SVO-style VO", vis)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

        prev_gray = gray
        prev_pts = next_good
        frame_idx += 1
finally:
    picam2.stop()
    cv2.destroyAllWindows()
