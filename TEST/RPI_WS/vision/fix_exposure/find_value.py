import picamera2
import time

camera = picamera2.Picamera2()
config = camera.create_preview_configuration(main={'format': 'RGB888', 'size': (640, 480)})
camera.configure(config)
camera.start()

# Warm up to let Auto-Exposure settle on the "Good" view
print("Pointing at target object... waiting 2 seconds...")
time.sleep(2)

# Get current metadata
metadata = camera.capture_metadata()

# Extract the values you need
# ExposureTime is in microseconds (e.g., 20000 = 1/50th second)
shutter = metadata['ExposureTime'] 
# AnalogueGain is effectively ISO (1.0 = Low ISO, 8.0 = High ISO)
gain = metadata['AnalogueGain'] 
# ColourGains controls White Balance (Red, Blue)
awb = metadata['ColourGains']

print("\n--- RECORD THESE NUMBERS ---")
print(f"Shutter (ExposureTime): {shutter}")
print(f"ISO (AnalogueGain):     {gain}")
print(f"AWB (ColourGains):      {awb}")
print("----------------------------")

camera.stop()
