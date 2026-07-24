"""
CameraServer: Real-time vision processing server for ArUco marker and color shape detection.
Architecture:
  - Captures video frames from Picamera2 on Raspberry Pi
  - Detects ArUco markers with 6x6 dictionary for 3D pose estimation (position & heading)
  - Detects colored shapes (triangles, rectangles, stars, circles) via HSV color thresholding
  - Publishes detection data as JSON on port 5557 and live video stream as JPEG on port 5558
  - Uses ZMQ PUB sockets for fast asynchronous broadcasting to multiple clients
  
Key Features:
  - 3D pose estimation requires camera calibration (calibration_data.npz)
  - Robust heading calculation immune to gimbal lock using Z-axis projection method
  - Morphological operations for noise reduction in color detection
  - Low-latency JPEG compression (quality 50) for real-time video streaming
"""
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
MARKER_SIZE = 0.065 # 6.5 cm

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
MY_AWB = (1.4530320167541504, 1.8029404878616333) 

COLOR_RANGES = {
    'RED'       : { 'lower1': (0, 100, 100),    'upper1': (10, 255, 255),   'lower2': (170, 100, 100),  'upper2': (179, 255, 255)},
    'GREEN'     : { 'lower1': (40, 80, 0),   'upper1': (90, 255, 255) },
    'BLUE'      : { 'lower1': (100,190,20),      'upper1': (120,255,255) },
    # 'PURPLE'    : { 'lower1': (120, 100, 100),  'upper1': (165, 255, 255) }
    'PURPLE'    : { 'lower1': (125, 80, 25),  'upper1': (140, 120, 255) }
}

CALIB_FILE = "calibration_data.npz"
ARUCO_DICT_TYPE = cv2.aruco.DICT_6X6_250

class CameraServer:
    def __init__(self):
        """
        Initialize camera, ZMQ sockets, ArUco detector, and load calibration data.
        
        - Creates two ZMQ PUB sockets: one for JSON detection data (port 5557),
          one for JPEG video frames (port 5558)
        - Configures Picamera2 with manual exposure, gain, and white balance
        - Loads camera intrinsics from calibration_data.npz if available
        - Sets up ArUco detector with DICT_6X6_250 dictionary
        
        Raises:
            RuntimeError: If camera initialization fails or ports cannot bind
        """
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
        
        self.mtx, self.dist = None, None
        
        if os.path.exists(CALIB_FILE):
            data = np.load(CALIB_FILE)
            self.mtx = data['mtx']
            self.dist = data['dist']
            logger.info("Loaded camera calibration data.")
        else:
            logger.warning("No calibration data found. ArUco pose estimation will be skipped.")

    def rotationMatrixToEulerAngles(self, R):
        """
        Convert 3x3 rotation matrix to Euler angles (roll, pitch, yaw in radians).
        This method has gimbal lock singularity at pitch=±90°.
            
        Returns:
            np.ndarray: [x, y, z] Euler angles where x=roll, y=pitch, z=yaw         
        """
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
    
    def get_heading_robust(self, R):
        """
        Calculate marker heading (yaw) by projecting marker's Z-axis onto camera's horizontal plane.
        This method is immune to gimbal lock that occurs at ±90° pitch in standard Euler decomposition.

        Returns:
            float: Heading angle in radians, range [-PI, PI], where 0 = camera forward direction
            
        Algorithm:
            1. Extract marker's Z-axis as seen from camera (R[:, 2])
            2. Project onto X-Z horizontal plane: (z_axis_x, z_axis_z)
            3. Calculate angle using atan2 with offset correction
        """
        
        # We assume the 'Forward' direction of the tag is its Z-axis (Column 2).
        z_axis_x = R[0, 2]
        z_axis_z = R[2, 2]
        
        # Calculate angle of that vector in the X-Z plane
        heading = math.atan2(z_axis_x, z_axis_z)

        if(heading > 0) :
            heading = heading - math.pi
        else :
            heading = heading + math.pi
        
        return heading

    def create_color_mask(self, hsv, color_name, limits):
        """
        Create binary mask for a specific color in HSV space with morphological filtering.
        """
        mask = cv2.inRange(hsv, limits['lower1'], limits['upper1'])
        if 'lower2' in limits:
            mask2 = cv2.inRange(hsv, limits['lower2'], limits['upper2'])
            mask = cv2.add(mask, mask2)
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return mask

    def detect_color_shapes(self, hsv_image):
        """
        Detect colored shapes (triangles, rectangles, stars, circles) in HSV image.
        
        Args:
            hsv_image (np.ndarray): Image in HSV color space
                
        Algorithm:
            1. For each color in COLOR_RANGES, create binary mask
            2. Find contours and filter by minimum area (1100 pixels)
            3. Approximate contour polygon and classify by vertex count:
               - 3 vertices = Triangle
               - 4 vertices = Rectangle
               - 8 vertices = Circle
               - 10 vertices = Star
            4. Calculate centroid and bounding box for each shape
        """
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
        """
        Detect ArUco markers and estimate 3D pose (position + heading) if calibration available.
        
        Args:
            image (np.ndarray): RGB/BGR image frame
            gray_image (np.ndarray): Grayscale version of image
            
        Returns:
            list: List of detected markers, each as dict:
                {
                    "type": "aruco",
                    "id": marker_id (int),
                    "center": [pixel_x, pixel_y],
                    "corners": [[x,y], ...] (4 corner points),
                    "rvec": [rx, ry, rz] (rotation vector),
                    "tvec": [tx, ty, tz] (translation vector in meters),
                    "yaw": heading in degrees (0-360),
                    "distance": distance from camera in meters,
                    "draw_rvec": rvec (numpy, for visualization),
                    "draw_tvec": tvec (numpy, for visualization)
                }
                If no calibration: returns 2D detection only (id, center, corners)
                
        Algorithm:
            1. Use cv2.aruco.ArucoDetector to find all markers
            2. If calibration exists, estimate 3D pose using cv2.solvePnP with IPPE_SQUARE method
            3. Convert rotation vector to rotation matrix
            4. Calculate yaw using robust heading method (immune to gimbal lock)
            5. Separate numpy arrays (with 'draw_' prefix) for visualization from JSON output
        """
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

                    success, rvec, tvec = cv2.solvePnP(
                        obj_points, corner, self.mtx, self.dist,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE
                    )
                    
                    if success:
                        pose_found = True
                        
                        rmat, _ = cv2.Rodrigues(rvec)
                        angles = self.rotationMatrixToEulerAngles(rmat)
                        yaw = self.get_heading_robust(rmat)
                        
                        detected.append({
                            "type": "aruco",
                            "id": marker_id,
                            "center": [center_x, center_y],
                            "corners": corner.tolist(),
                            "rvec": rvec.flatten().tolist(),
                            "tvec": tvec.flatten().tolist(),
                            # "yaw": math.degrees(angles[1]),
                            "yaw": math.degrees(yaw),
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
        """
        Main vision loop: capture frames, process, and broadcast detections & video stream.
        
        Loop sequence (every ~100ms at 10 FPS):
            1. Capture RGB frame and convert to HSV/grayscale
            2. Detect colored shapes and ArUco markers in parallel
            3. Draw overlays on frame (bounding boxes, axes, labels)
            4. Publish JSON detection data (shapes + markers metadata)
            5. Publish JPEG-compressed video stream for live monitoring
            6. Log findings (INFO on detection, DEBUG heartbeat every 1 second)
            
        Graceful shutdown:
            - Catches KeyboardInterrupt (Ctrl+C) to close camera and sockets
            - Continues on frame processing errors to prevent crash (critical for robotic systems)
        """

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
                
                time.sleep(0.1) # Cap at ~10 FPS

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