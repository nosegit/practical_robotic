import picamera2
import time
import cv2

# --- REPLACE THESE WITH YOUR NUMBERS FROM STEP 1 ---
MY_SHUTTER = 23846      # Example: 20ms
MY_GAIN = 2.0           # Example: Low ISO
MY_AWB = (1.4530320167541504, 1.8029404878616333)     # Example: (Red_Gain, Blue_Gain)

camera = picamera2.Picamera2()
config = camera.create_preview_configuration(main={'format': 'RGB888', 'size': (640, 480)})
camera.configure(config)
camera.start()

# --- APPLY MANUAL CONTROLS ---
# AeEnable=False: Turn off Auto Exposure
# AwbEnable=False: Turn off Auto White Balance
controls = {
    "AeEnable": False,
    "ExposureTime": MY_SHUTTER,
    "AnalogueGain": MY_GAIN,
    "AwbEnable": False,
    "ColourGains": MY_AWB
}

camera.set_controls(controls)
print("🔒 Exposure & Color Locked!")

while True:
    image = camera.capture_array("main")
    # Convert to BGR for OpenCV display (since we are using RGB888 config)
    # image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    
    cv2.imshow("Manual Mode", image)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

camera.stop()
cv2.destroyAllWindows()
