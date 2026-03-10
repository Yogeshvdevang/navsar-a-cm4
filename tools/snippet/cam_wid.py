import cv2
import numpy as np
from picamera2 import Picamera2

# Initialize the camera
picam2 = Picamera2()

# Configure for Monochrome/Grayscale at high speed
# 'YUV420' is efficient for processing grayscale frames in OpenCV
config = picam2.create_video_configuration(main={"format": "YUV420", "size": (640, 400)})
picam2.configure(config)
picam2.start()

print("OV9281 Live Feed Started. Press 'q' to quit.")

try:
    while True:
        # Capture frame as a numpy array
        frame = picam2.capture_array()

        # In YUV420, the first 'layer' is the Grayscale (Luma) image
        # This is much faster for Odometry than converting color images
        gray_frame = frame[:400, :640] 

        # Show the frame
        cv2.imshow("OV9281 Odometry Feed", gray_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    picam2.stop()
    cv2.destroyAllWindows()