import picamera2
import time
import cv2
import os
from datetime import datetime

# --- CONFIGURATION ---
SAVE_FOLDER = "image"

# 1. Create the folder if it doesn't exist
if not os.path.exists(SAVE_FOLDER):
    os.makedirs(SAVE_FOLDER)
    print(f"📂 Created folder: {SAVE_FOLDER}")

# 2. Initialise Camera
camera = picamera2.Picamera2()
config = camera.create_preview_configuration(main={'format': 'RGB888', 'size': (640, 480)})
camera.configure(config)
camera.start()

time.sleep(0.1)

print(f"📷 Camera Running... Images will be saved to '/{SAVE_FOLDER}'")
print("Press 's' to SAVE an image")
print("Press 'q' to QUIT")

try:
    while True:
        # Capture
        image = camera.capture_array("main")

        # Display
        cv2.imshow("Frame", image)

        key = cv2.waitKey(1) & 0xFF

        # --- SAVE LOGIC ---
        if key == ord("s"):
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
            # Combine folder name and filename
            # Example: image/capture_20231120_103005.jpg
            filename = f"capture_{timestamp}.jpg"
            full_path = os.path.join(SAVE_FOLDER, filename)
            
            cv2.imwrite(full_path, image)
            print(f"✅ Saved to: {full_path}")

        elif key == ord("q"):
            break

except Exception as e:
    print(f"Error: {e}")

finally:
    camera.stop()
    cv2.destroyAllWindows()
