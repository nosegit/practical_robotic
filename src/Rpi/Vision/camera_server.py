import picamera2
import cv2
import numpy as np
import zmq
import json
import time
import os
import logging
import math

# --- CONFIGURATION ---
PUB_PORT = 5557
TOPIC = "VISION"

# Marker size in Meters (CRITICAL for distance accuracy)
MARKER_SIZE = 0.06 

# Smoothing Factor (0.0 to 1.0)
# Higher = More jitter, faster response. Lower = Smoother, slightly more lag.
SMOOTHING_ALPHA = 0.5 

# --- LOGGING SETUP ---
# 1. Set Root logger to DEBUG so it accepts ALL messages initially
logging.basicConfig(
    level=logging.DEBUG, 
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S'
)

# 2. Configure OUR logger
logger = logging.getLogger("VisionServer")
logger.setLevel(logging.DEBUG)

# 3. Silence specific noisy libraries
logging.getLogger("picamera2").setLevel(logging.WARNING)
logging.getLogger("PIL").setLevel(logging.WARNING)

# Camera Settings
MY_SHUTTER = 23846      
MY_GAIN = 2.0           
MY_AWB = (1.453, 1.803) 

# Color Ranges
COLOR_RANGES = {
    'RED':    {'lower1': (0, 100, 100), 'upper1': (10, 255, 255), 'lower2': (170, 100, 100), 'upper2': (179, 255, 255)},
    'GREEN':  {'lower1': (40, 60, 60),   'upper1': (90, 255, 255)},
    'BLUE':   {'lower1': (100, 150, 0),  'upper1': (140, 255, 255)},
    'YELLOW': {'lower1': (20, 100, 100), 'upper1': (30, 255, 255)}
}

CALIB_FILE = "calibration_data.npz"

# Use 6x6_250 Dictionary
ARUCO_DICT_TYPE = cv2.aruco.DICT_6X6_250

class CameraServer:
    def __init__(self):
        # 1. Setup ZeroMQ
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind(f"tcp://*:{PUB_PORT}")
        logger.info(f"Vision Server publishing on port {PUB_PORT}")

        # 2. Setup Camera
        logger.info("Initializing Camera...")
        self.camera = picamera2.Picamera2()
        config = self.camera.create_preview_configuration(main={'format': 'RGB888', 'size': (640, 480)})
        self.camera.configure(config)
        self.camera.start()
        
        # Apply Manual Controls
        self.camera.set_controls({
            "AeEnable": False,
            "ExposureTime": MY_SHUTTER,
            "AnalogueGain": MY_GAIN,
            "AwbEnable": False,
            "ColourGains": MY_AWB
        })
        time.sleep(1) # Warmup

        # 3. Setup ArUco
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_TYPE)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # History for Smoothing & Ambiguity Check
        self.pose_history = {} 

        # Load Calibration
        self.mtx, self.dist = None, None
        if os.path.exists(CALIB_FILE):
            data = np.load(CALIB_FILE)
            self.mtx = data['mtx']
            self.dist = data['dist']
            logger.info("Loaded camera calibration data.")
        else:
            logger.warning("No calibration data found. ArUco pose estimation will be skipped.")

    def rotationMatrixToEulerAngles(self, R):
        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        singular = sy < 1e-6
        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0
        return np.array([x, y, z])

    def create_color_mask(self, hsv, color_name, limits):
        mask = cv2.inRange(hsv, limits['lower1'], limits['upper1'])
        if 'lower2' in limits:
            mask2 = cv2.inRange(hsv, limits['lower2'], limits['upper2'])
            mask = cv2.add(mask, mask2)
        
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return mask

    def detect_color_shapes(self, hsv_image):
        detected = []
        for color_name, limits in COLOR_RANGES.items():
            mask = self.create_color_mask(hsv_image, color_name, limits)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for c in contours:
                area = cv2.contourArea(c)
                if area < 1100: continue # Ignore noise

                peri = cv2.arcLength(c, True)
                epsilon = 0.025 * peri 
                approx = cv2.approxPolyDP(c, epsilon, True)
                vertices = len(approx)

                shape_type = "Unknown"
                if vertices == 3:
                    shape_type = "Triangle"
                elif vertices == 4:
                    shape_type = "Rectangle"
                elif vertices == 10: 
                    shape_type = "Star"
                elif vertices == 8:
                    shape_type = "Circle"

                M = cv2.moments(c)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                else:
                    cX, cY = 0, 0

                detected.append({
                    "type": "shape",
                    "label": shape_type,
                    "color": color_name,
                    "center": [cX, cY],
                    "area": area
                })
        return detected

    def detect_aruco(self, image, gray_image):
        detected = []
        corners, ids, rejected = self.detector.detectMarkers(gray_image)
        
        if ids is not None and len(ids) > 0:
            if self.mtx is not None:
                # Prepare object points for solvePnP (Square marker in 3D)
                # Top-Left, Top-Right, Bottom-Right, Bottom-Left
                obj_points = np.array([
                    [-MARKER_SIZE/2,  MARKER_SIZE/2, 0],
                    [ MARKER_SIZE/2,  MARKER_SIZE/2, 0],
                    [ MARKER_SIZE/2, -MARKER_SIZE/2, 0],
                    [-MARKER_SIZE/2, -MARKER_SIZE/2, 0]
                ], dtype=np.float32)

                for i in range(len(ids)):
                    marker_id = int(ids[i][0])
                    corner = corners[i][0] # 2D corners of this marker
                    
                    # --- AMBIGUITY FIX & SMOOTHING ---
                    success = False
                    rvec, tvec = None, None

                    if marker_id in self.pose_history:
                        old_rvec, old_tvec = self.pose_history[marker_id]
                        
                        # Use previous pose as guess to prevent flipping
                        success, rvec, tvec = cv2.solvePnP(
                            obj_points, corner, self.mtx, self.dist,
                            rvec=old_rvec, tvec=old_tvec,
                            useExtrinsicGuess=True,
                            flags=cv2.SOLVEPNP_ITERATIVE
                        )
                        
                        if success:
                            # Apply Smoothing Filter
                            rvec = SMOOTHING_ALPHA * rvec + (1 - SMOOTHING_ALPHA) * old_rvec
                            tvec = SMOOTHING_ALPHA * tvec + (1 - SMOOTHING_ALPHA) * old_tvec
                            
                    else:
                        # First time detection (No guess available)
                        # IPPE_SQUARE is robust for initial planar markers
                        success, rvec, tvec = cv2.solvePnP(
                            obj_points, corner, self.mtx, self.dist,
                            flags=cv2.SOLVEPNP_IPPE_SQUARE
                        )
                    
                    if success:
                        # Update History
                        self.pose_history[marker_id] = (rvec, tvec)

                        center_x = int(np.mean(corner[:, 0]))
                        center_y = int(np.mean(corner[:, 1]))
                        
                        rmat, _ = cv2.Rodrigues(rvec)
                        angles = self.rotationMatrixToEulerAngles(rmat)
                        
                        pitch = math.degrees(angles[0])
                        yaw = math.degrees(angles[1])
                        roll = math.degrees(angles[2])
                        
                        obj = {
                            "type": "aruco",
                            "id": marker_id,
                            "center": [center_x, center_y],
                            "corners": corner.tolist(),
                            "rvec": rvec.flatten().tolist(), # Flatten for JSON
                            "tvec": tvec.flatten().tolist(), # Flatten for JSON
                            "rotation_matrix": rmat.tolist(),
                            "yaw": yaw,
                            "pitch": pitch,
                            "roll": roll,
                            "distance": float(np.linalg.norm(tvec))
                        }
                        detected.append(obj)
            else:
                for i in range(len(ids)):
                    marker_id = int(ids[i][0])
                    corner = corners[i][0]
                    center_x = int(np.mean(corner[:, 0]))
                    center_y = int(np.mean(corner[:, 1]))
                    detected.append({
                        "type": "aruco",
                        "id": marker_id,
                        "center": [center_x, center_y],
                        "corners": corner.tolist()
                    })

        return detected

    def run(self):
        logger.info("Starting Vision Loop...")
        frame_counter = 0
        last_log_time = time.time()
        
        try:
            while True:
                # 1. Capture
                image = self.camera.capture_array("main")
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                
                # 2. Process
                shapes = self.detect_color_shapes(hsv)
                markers = self.detect_aruco(image, gray)
                
                # 3. Publish
                payload = {
                    "timestamp": time.time(),
                    "shapes": shapes,
                    "aruco": markers,
                    "count_shapes": len(shapes),
                    "count_aruco": len(markers)
                }
                self.socket.send_string(f"{TOPIC} {json.dumps(payload)}")
                
                # 4. Logging & Stats
                frame_counter += 1
                current_time = time.time()
                
                # INFO: Log immediately if we see something
                if len(shapes) > 0 or len(markers) > 0:
                     logger.info(f"[FOUND] Shapes: {len(shapes)} | ArUco: {len(markers)}")
                
                # DEBUG: Heartbeat every 1 second (Visible pulse)
                elif (current_time - last_log_time >= 1.0):
                    fps = frame_counter / (current_time - last_log_time)
                    logger.debug(f"[STATUS] Alive | FPS: {fps:.1f} | Scanning...")
                    frame_counter = 0
                    last_log_time = current_time
                
                time.sleep(0.05) # Cap at ~20 FPS

        except KeyboardInterrupt:
            logger.info("Stopping Camera Server...")
            self.camera.stop()
            self.socket.close()
            self.context.term()
        except Exception as e:
            logger.exception("Critical Error in Vision Loop")

if __name__ == "__main__":
    server = CameraServer()
    server.run()