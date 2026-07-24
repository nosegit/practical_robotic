# Machine vision lab - Practical Robotics
# test_camera.py: Grab the camera framebuffer in OpenCV and display using cv2.imshow


# Import the necessary packages
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
# Allow the camera time to warm up
time.sleep(0.1)

blueHueMin = 100
blueSatMin = 190
blueValMin = 20

blueHueMax = 120
blueSatMax = 255
blueValMax = 255
# capture frames from the camera
while True:

	image = camera.capture_array("main")


	image_rotate = cv2.rotate(image,cv2.ROTATE_180)


	# blue masking in hse (hue[0,179], saturation[0,255], value)
	blueMin = (blueHueMin, blueSatMin, blueValMin)
	blueMax = (blueHueMax, blueSatMax, blueValMax)

	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(hsv,blueMin,blueMax)

	masked_image = cv2.bitwise_and(image, image, mask=mask)


	# Show the frame
	# cv2.imshow("Frame", image)
	cv2.imshow("blue Frame", masked_image)

	key = cv2.waitKey(1) & 0xFF
	if key == 27 : # esc
		break
	elif  key == ord("q"):
		blueHueMin = blueHueMin - 5
		print(f"blueHueMin : {blueHueMin}")
	elif key == ord("e"):
		blueHueMin = blueHueMin + 5
		print(f"blueHueMin : {blueHueMin}")
	elif key == ord("a"): 
		blueSatMin = blueSatMin - 5
		print(f"blueSatMin : {blueSatMin}")
	elif key == ord("d"): 
		blueSatMin = blueSatMin + 5 
		print(f"blueSatMin : {blueSatMin}")
	elif key == ord("z"): 
		blueValMin = blueValMin - 5
		print(f"blueValMin : {blueValMin}")
	elif key == ord("c"): 
		blueValMin = blueValMin + 5
		print(f"blueValMin : {blueValMin}")
	elif  key == ord("t"):
		blueHueMax = blueHueMax - 5
		print(f"blueHueMax : {blueHueMax}")
	elif key == ord("u"):
		blueHueMax = blueHueMax + 5
		print(f"blueHueMax : {blueHueMax}")
	elif key == ord("g"): 
		blueSatMax = blueSatMax - 5
		print(f"blueSatMax : {blueSatMax}")
	elif key == ord("j"): 
		blueSatMax = blueSatMax + 5 
		print(f"blueSatMax : {blueSatMax}")
	elif key == ord("v"): 
		blueValMax = blueValMax - 5
		print(f"blueValMax : {blueValMax}")
	elif key == ord("n"): 
		blueValMax = blueValMax + 5
		print(f"blueValMax : {blueValMax}")
	elif key == ord("p"):
		print(f"blueMin : ({blueHueMin}, {blueSatMin}, {blueValMin})")
		print(f"blueMax : ({blueHueMax}, {blueSatMax}, {blueValMax})")



		
