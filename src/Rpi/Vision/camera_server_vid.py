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
PUB_PORT = 5557       # Port for JSON Data
VIDEO_PORT = 5558     # Port for JPEG Stream
TOPIC = "VISION"

# Marker size in Meters
MARKER_SIZE = 0.065
SMOOTHING_ALPHA = 0.7 

# --- LOGGING SETUP ---
logging.basicConfig(
    level=logging.DEBUG, 
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger("VisionServer")
logger.setLevel(logging.DEBUG)

# Silence noisy libraries
logging.getLogger("picamera2").setLevel(logging.WARNING)
logging.getLogger("PIL").setLevel(logging.WARNING)

# Camera Settings
MY_SHUTTER = 23846      
MY_GAIN = 2.0           
MY_AWB = (1.453, 1.803) 

COLOR_RANGES = {
    'RED':    {'lower1': (0, 100, 100), 'upper1': (10, 255, 255), 'lower2': (170, 100, 100), 'upper2': (179, 255, 255)},
    'GREEN':  {'lower1': (40, 60, 60),   'upper1': (90, 255, 255)},
    'BLUE':   {'lower1': (100, 150, 0),  'upper1': (140, 255, 255)},
    'YELLOW': {'lower1': (20, 100, 100), 'upper1': (30, 255, 255)}
}

CALIB_FILE = "calibration_data.npz"
ARUCO_DICT_TYPE = cv2.aruco.DICT_6X6_250

class CameraServer:
    def __init__(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind(f"tcp://*:{PUB_PORT}")
        
        self.video_socket = self.context.socket(zmq.PUB)
        self.video_socket.bind(f"tcp://*:{VIDEO_PORT}")
        
        logger.info(f"Vision Server publishing DATA on {PUB_PORT}, VIDEO on {VIDEO_PORT}")

        logger.info("Initializing Camera...")
        self.camera = picamera2.Picamera2()
        config = self.camera.create_preview_configuration(main={'format': 'RGB888', 'size': (640, 480)})
        self.camera.configure(config)
        self.camera.start()
        
        self.camera.set_controls({
            "AeEnable": False,
            "ExposureTime": MY_SHUTTER,
            "AnalogueGain": MY_GAIN,
            "AwbEnable": False,
            "ColourGains": MY_AWB
        })
        time.sleep(1) 

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_TYPE)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        self.pose_history = {} 
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
                if area < 1100: continue 

                peri = cv2.arcLength(c, True)
                epsilon = 0.025 * peri 
                approx = cv2.approxPolyDP(c, epsilon, True)
                vertices = len(approx)

                shape_type = "Unknown"
                if vertices == 3: shape_type = "Triangle"
                elif vertices == 4: shape_type = "Rectangle"
                elif vertices == 10: shape_type = "Star"
                elif vertices == 8: shape_type = "Circle"

                M = cv2.moments(c)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                else:
                    cX, cY = 0, 0
                
                # CRITICAL FIX: Ensure bbox is standard python int, not numpy int
                x, y, w, h = cv2.boundingRect(approx)
                bbox = (int(x), int(y), int(w), int(h))

                detected.append({
                    "type": "shape",
                    "label": shape_type,
                    "color": color_name,
                    "center": [cX, cY],
                    "area": float(area),
                    "bbox": bbox 
                })
        return detected

    def detect_aruco(self, image, gray_image):
        detected = []
        corners, ids, rejected = self.detector.detectMarkers(gray_image)
        
        if ids is not None and len(ids) > 0:
            # Prepare 3D points
            if self.mtx is not None:
                obj_points = np.array([
                    [-MARKER_SIZE/2,  MARKER_SIZE/2, 0],
                    [ MARKER_SIZE/2,  MARKER_SIZE/2, 0],
                    [ MARKER_SIZE/2, -MARKER_SIZE/2, 0],
                    [-MARKER_SIZE/2, -MARKER_SIZE/2, 0]
                ], dtype=np.float32)

            for i in range(len(ids)):
                marker_id = int(ids[i][0])
                corner = corners[i][0]
                center_x = int(np.mean(corner[:, 0]))
                center_y = int(np.mean(corner[:, 1]))
                
                pose_found = False
                
                # Try 3D Pose Estimation
                if self.mtx is not None:
                    success = False
                    rvec, tvec = None, None

                    if marker_id in self.pose_history:
                        old_rvec, old_tvec = self.pose_history[marker_id]
                        success, rvec, tvec = cv2.solvePnP(
                            obj_points, corner, self.mtx, self.dist,
                            rvec=old_rvec, tvec=old_tvec,
                            useExtrinsicGuess=True,
                            flags=cv2.SOLVEPNP_ITERATIVE
                        )
                        if success:
                            rvec = SMOOTHING_ALPHA * rvec + (1 - SMOOTHING_ALPHA) * old_rvec
                            tvec = SMOOTHING_ALPHA * tvec + (1 - SMOOTHING_ALPHA) * old_tvec
                    else:
                        success, rvec, tvec = cv2.solvePnP(
                            obj_points, corner, self.mtx, self.dist,
                            flags=cv2.SOLVEPNP_IPPE_SQUARE
                        )
                    
                    if success:
                        pose_found = True
                        self.pose_history[marker_id] = (rvec, tvec)
                        
                        rmat, _ = cv2.Rodrigues(rvec)
                        angles = self.rotationMatrixToEulerAngles(rmat)
                        
                        detected.append({
                            "type": "aruco",
                            "id": marker_id,
                            "center": [center_x, center_y],
                            "corners": corner.tolist(),
                            "rvec": rvec.flatten().tolist(),
                            "tvec": tvec.flatten().tolist(),
                            "yaw": math.degrees(angles[1]),
                            "distance": float(np.linalg.norm(tvec)),
                            "draw_rvec": rvec, # Numpy for drawing
                            "draw_tvec": tvec
                        })
                
                # Fallback to 2D if Pose failed or No Calibration
                if not pose_found:
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
        
        while True:
            try:
                # 1. Capture
                image = self.camera.capture_array("main")
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                
                # 2. Process
                shapes = self.detect_color_shapes(hsv)
                markers = self.detect_aruco(image, gray)
                
                # 3. Draw Overlays (Visual Debugging)
                display_img = image.copy()
                
                for s in shapes:
                    x, y, w, h = s['bbox']
                    cv2.rectangle(display_img, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    cv2.putText(display_img, f"{s['color']} {s['label']}", (x, y-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                for m in markers:
                    if 'draw_rvec' in m and self.mtx is not None:
                        cv2.drawFrameAxes(display_img, self.mtx, self.dist, 
                                        m['draw_rvec'], m['draw_tvec'], 0.05)
                        cv2.putText(display_img, f"ID:{m['id']} {m['distance']:.2f}m", 
                                  (m['center'][0], m['center'][1]), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                # 4. Publish JSON Data
                # CRITICAL: Remove 'draw_' keys containing numpy arrays to prevent JSON crash
                clean_markers = [{k: v for k, v in m.items() if not k.startswith('draw_')} for m in markers]
                
                payload = {
                    "timestamp": time.time(),
                    "shapes": shapes, 
                    "aruco": clean_markers,
                    "count_shapes": len(shapes),
                    "count_aruco": len(markers)
                }
                
                try:
                    json_str = json.dumps(payload)
                    self.socket.send_string(f"{TOPIC} {json_str}")
                except TypeError as e:
                    logger.error(f"JSON Serialization Failed: {e}")
                    # Don't crash, just skip sending this frame's JSON
                
                # 5. Publish VIDEO Stream (Compressed JPEG)
                # Lower quality to 50 for performance
                ret, jpg_buffer = cv2.imencode('.jpg', display_img, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
                if ret:
                    self.video_socket.send(jpg_buffer.tobytes())

                # 6. Logging
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
                self.video_socket.close()
                self.context.term()
                break
            except Exception as e:
                # Loop Protection: Log error but continue!
                logger.error(f"Runtime Error in Frame: {e}")
                time.sleep(0.1)

if __name__ == "__main__":
    server = CameraServer()
    server.run()