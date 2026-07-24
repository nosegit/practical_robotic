import picamera2
import cv2
from cv2 import aruco
import numpy as np
import time

def get_aruco_data(frame, aruco_dict, parameters):
    """
    Detects ArUco markers and returns a list of dictionaries.
    Output format: [{'id': 10, 'location': (320, 240)}, ...]
    """
    # 1. Convert to Grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 2. Detect Markers
    corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)

    results = []

    # 3. Process if markers found
    if ids is not None:
        # Flatten the IDs list (comes as [[1], [2]] -> [1, 2])
        ids = ids.flatten()

        for i, marker_id in enumerate(ids):
            # Get the 4 points for the current marker
            # corners[i][0] returns shape (4, 2) -> [[x1,y1], [x2,y2], [x3,y3], [x4,y4]]
            marker_points = corners[i][0].astype(int)

            # --- 1. GET BOUNDING BOX (x, y, w, h) ---
            # cv2.boundingRect automatically finds the top-left x,y and the width/height
            # regardless of how the marker is rotated.
            x, y, w, h = cv2.boundingRect(marker_points)

            # --- 2. CALCULATE CENTER (Centroid) ---
            # The most accurate center is the average of all 4 corners
            c_x = int(np.mean(marker_points[:, 0]))
            c_y = int(np.mean(marker_points[:, 1]))

            # --- 3. CREATE DICTIONARY ---
            data = {
                "id": int(marker_id),
                "center": (c_x, c_y),
                "location": (x, y, w, h) # Matches your requirement
            }
            results.append(data)

    return results  # Return corners too in case you want to draw them later

# Initialise the camera and create a reference to it
camera = picamera2.Picamera2()
config = camera.create_preview_configuration(main={'format': 'RGB888', 'size' : (640,480)})
camera.configure(config)
camera.start()

# Allow the camera time to warm up
time.sleep(0.1)

# Setup aruco dictionary and parameters
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
aruco_parameters = aruco.DetectorParameters_create()


if __name__ == "__main__":
    while True:
        image = camera.capture_array("main")
        
        aruco_data = get_aruco_data(image, aruco_dict, aruco_parameters)

        for data in aruco_data:
            (x, y, w, h) = data["location"]
            cv2.rectangle(image,(x,y), (x+w, y+h), (0,0,255),1)
            # cv2.circle(image, data['center'], 5, (0,0,255), -1 )
            cv2.putText(image,f"{data['id']}",data["center"], cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)

        cv2.imshow("Aruco Detection", image)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break

