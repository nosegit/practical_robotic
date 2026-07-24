"""
CameraCalibration: Interactive tool for calibrating Raspberry Pi camera intrinsics using chessboard pattern.
Captures multiple chessboard images, computes camera matrix (mtx) and distortion coefficients (dist),
saves to calibration_data.npz for use in ArUco 3D pose estimation.

Workflow: Capture 15+ chessboard frames (press 's'), finish with 'q', calibration auto-computes.
Output: calibration_data.npz containing mtx and dist for cv2.solvePnP and cv2.drawFrameAxes.
"""
import cv2
import numpy as np
import time
import picamera2

# --- USER CONFIGURATION ---
# Replace these with your tuned values if they change
MY_SHUTTER = 23846      # in microseconds
MY_GAIN = 2.0
MY_AWB = (1.4530320167541504, 1.8029404878616333) # (Red, Blue)

# Calibration Config
CHESSBOARD_SIZE = (9, 6) # Inner corners of chessboard pattern
SQUARE_SIZE = 0.019      # Physical size of one square in meters
OUTPUT_FILE = "calibration_data.npz"

# --- CAMERA SETUP ---
print("Initializing PiCamera2...")
camera = picamera2.Picamera2()
config = camera.create_preview_configuration(main={'format': 'RGB888', 'size': (640, 480)})
camera.configure(config)
camera.start()

# Apply Manual Controls to ensure consistent lighting across captures
controls = {
    "AeEnable": False,
    "ExposureTime": MY_SHUTTER,
    "AnalogueGain": MY_GAIN,
    "AwbEnable": False,
    "ColourGains": MY_AWB
}
camera.set_controls(controls)
print("Camera configured with manual settings.")

# --- CALIBRATION SETUP ---
# OpenCV corner refinement: max 30 iterations, precision 0.001 pixels
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Generate 3D world points for a flat chessboard (Z=0)
objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
objp = objp * SQUARE_SIZE

# Storage for detected points across multiple images
objpoints = [] # 3D world points
imgpoints = [] # 2D image points

print("--- Camera Calibration ---")
print("Press 's' to save a frame (capture at least 15).")
print("Press 'q' to finish and calculate.")

count = 0

try:
    while True:
        # Capture frame using PiCamera2
        frame = camera.capture_array("main")

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Find chess board corners (OpenCV detects inner corners)
        ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)

        display_frame = frame.copy()
        if ret == True:
            # Visualize detected corners on display frame
            cv2.drawChessboardCorners(display_frame, CHESSBOARD_SIZE, corners, ret)

        cv2.imshow('Calibration', display_frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('s') and ret == True:
            # Refine corner positions to sub-pixel accuracy
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            objpoints.append(objp)
            imgpoints.append(corners2)
            count += 1
            print(f"Captured Image {count}")
            # Flash effect for user feedback
            cv2.imshow('Calibration', np.zeros_like(display_frame))
            cv2.waitKey(50)
            
        elif key == ord('q'):
            break
finally:
    camera.stop()
    cv2.destroyAllWindows()

# --- CALCULATE & SAVE ---
if count > 10:
    print("Calibrating... please wait.")
    # Compute camera matrix (mtx) and distortion coefficients (dist)
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    
    print("Calibration successful. Saving to", OUTPUT_FILE)
    np.savez(OUTPUT_FILE, mtx=mtx, dist=dist)
    
    # Calculate reprojection error: should be < 1.0 for good calibration
    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
        mean_error += error
    print(f"Total error: {mean_error/len(objpoints)} (Ideally < 1.0)")
else:
    print("Not enough points captured (Need > 10).")