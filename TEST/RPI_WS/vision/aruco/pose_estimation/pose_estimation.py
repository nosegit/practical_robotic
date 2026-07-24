import cv2
import numpy as np
import math
import picamera2

# --- USER CONFIGURATION ---
CALIB_FILE = "calibration_data.npz"
MARKER_SIZE = 0.06  # Meters (Measure your specific tag!)

# --- RESOLUTION SETTINGS ---
CAMERA_RES = (640, 480) 

# SMOOTHING FACTOR (0.0 to 1.0)
SMOOTHING_ALPHA = 0.7 

# Camera Settings
MY_SHUTTER = 23846 
MY_GAIN = 2.0 
MY_AWB = (1.4530320167541504, 1.8029404878616333)

# Wall Angles (Tag ID: Angle in degrees)
TAG_REAL_WORLD_ANGLES = {
    0: 0,    
    1: 90,   
    2: 180,  
    3: 270   
}

# --- LOAD CALIBRATION ---
try:
    with np.load(CALIB_FILE) as X:
        camera_matrix, dist_coeffs = [X[i] for i in ('mtx', 'dist')]
    print(f"Calibration loaded. Matrix shape: {camera_matrix.shape}")
except:
    print("Error: Run calibrate_camera.py first!")
    exit()

# --- CAMERA SETUP ---
print(f"Starting Camera at {CAMERA_RES}...")
camera = picamera2.Picamera2()
config = camera.create_preview_configuration(main={'format': 'RGB888', 'size': CAMERA_RES})
camera.configure(config)
camera.start()
camera.set_controls({
    "AeEnable": False,
    "ExposureTime": MY_SHUTTER,
    "AnalogueGain": MY_GAIN,
    "AwbEnable": False,
    "ColourGains": MY_AWB
})

# --- ARUCO SETUP ---
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters()
parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

# Define the 3D coordinates of the marker corners
marker_3d_edges = np.array([
    [-MARKER_SIZE/2.0,  MARKER_SIZE/2.0, 0],
    [ MARKER_SIZE/2.0,  MARKER_SIZE/2.0, 0],
    [ MARKER_SIZE/2.0, -MARKER_SIZE/2.0, 0],
    [-MARKER_SIZE/2.0, -MARKER_SIZE/2.0, 0]
], dtype=np.float32)

def rotationMatrixToEulerAngles(R):
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    if not sy < 1e-6:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0
    return np.array([x, y, z])

# History for smoothing and tracking
pose_history = {}

print("Pose Estimation Running. Press 'q' to quit.")

try:
    while True:
        frame = camera.capture_array()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = detector.detectMarkers(gray)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            
            for i in range(len(ids)):
                current_id = ids[i][0]
                
                # --- AMBIGUITY FIX: Use Extrinsic Guess ---
                # If we detected this tag recently, use its last known position 
                # as a "guess" for the solver. This prevents it from flipping 90 degrees.
                if current_id in pose_history:
                    old_rvec, old_tvec = pose_history[current_id]
                    
                    # Use SOLVEPNP_ITERATIVE with useExtrinsicGuess=True
                    # This forces the solver to find a solution close to the previous one
                    ret, rvec, tvec = cv2.solvePnP(
                        marker_3d_edges, 
                        corners[i], 
                        camera_matrix, 
                        dist_coeffs, 
                        rvec=old_rvec, # Pass previous values
                        tvec=old_tvec, 
                        useExtrinsicGuess=True, 
                        flags=cv2.SOLVEPNP_ITERATIVE
                    )
                else:
                    # First time seeing this tag? Use IPPE_SQUARE to find it initially.
                    # We cannot use a "guess" because we don't have one yet.
                    ret, rvec, tvec = cv2.solvePnP(
                        marker_3d_edges, 
                        corners[i], 
                        camera_matrix, 
                        dist_coeffs, 
                        flags=cv2.SOLVEPNP_IPPE_SQUARE
                    )

                # --- APPLY SMOOTHING ---
                if current_id in pose_history:
                    old_rvec, old_tvec = pose_history[current_id]
                    rvec = SMOOTHING_ALPHA * rvec + (1 - SMOOTHING_ALPHA) * old_rvec
                    tvec = SMOOTHING_ALPHA * tvec + (1 - SMOOTHING_ALPHA) * old_tvec
                
                pose_history[current_id] = (rvec, tvec)
                # -----------------------

                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.1)
                
                rmat, _ = cv2.Rodrigues(rvec)
                angles = rotationMatrixToEulerAngles(rmat)
                yaw_deg = math.degrees(angles[1]) 

                print(yaw_deg)
                
                dist = tvec[2][0] 
                
                # if current_id in TAG_REAL_WORLD_ANGLES:
                #     known_angle = TAG_REAL_WORLD_ANGLES[current_id]
                #     robot_heading = (known_angle - yaw_deg) % 360
                    
                #     text = f"H: {int(robot_heading)}deg D: {dist:.2f}m"
                #     cv2.putText(frame, text, (10, 30 + i*40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                #     print(f"ID:{current_id} RobotHeading:{int(robot_heading)}")

        # Clear history if no tags seen (optional, keeps memory clean)
        # In a real robot, you might want to keep history for a few seconds 
        # even if tag is lost briefly.
        if ids is None:
             pose_history = {}

        cv2.imshow('Robot Vision', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    camera.stop()
    cv2.destroyAllWindows()