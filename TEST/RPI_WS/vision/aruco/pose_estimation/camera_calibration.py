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
CHESSBOARD_SIZE = (9, 6) # Inner corners
SQUARE_SIZE = 0.019      # Meters
OUTPUT_FILE = "calibration_data.npz"

# --- CAMERA SETUP ---
print("Initializing PiCamera2...")
camera = picamera2.Picamera2()
config = camera.create_preview_configuration(main={'format': 'RGB888', 'size': (640, 480)})
camera.configure(config)
camera.start()

# Apply Manual Controls
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
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
objp = objp * SQUARE_SIZE

objpoints = [] 
imgpoints = [] 

print("--- Camera Calibration ---")
print("Press 's' to save a frame (capture at least 15).")
print("Press 'q' to finish and calculate.")

count = 0

try:
    while True:
        # Capture frame using PiCamera2
        frame = camera.capture_array("main")

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Find chess board corners
        ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)

        display_frame = frame.copy()
        if ret == True:
            cv2.drawChessboardCorners(display_frame, CHESSBOARD_SIZE, corners, ret)

        cv2.imshow('Calibration', display_frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('s') and ret == True:
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            objpoints.append(objp)
            imgpoints.append(corners2)
            count += 1
            print(f"Captured Image {count}")
            # Flash effect
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
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    
    print("Calibration successful. Saving to", OUTPUT_FILE)
    np.savez(OUTPUT_FILE, mtx=mtx, dist=dist)
    
    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
        mean_error += error
    print(f"Total error: {mean_error/len(objpoints)} (Ideally < 1.0)")
else:
    print("Not enough points captured (Need > 10).")