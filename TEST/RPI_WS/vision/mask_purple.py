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

purpleHueMin = 130
purpleSatMin = 80
purpleValMin = 25

purpleHueMax = 140
purpleSatMax = 120
purpleValMax = 255
# capture frames from the camera
while True:

	image = camera.capture_array("main")


	image_rotate = cv2.rotate(image,cv2.ROTATE_180)


	# purple masking in hse (hue[0,179], saturation[0,255], value)
	purpleMin = (purpleHueMin, purpleSatMin, purpleValMin)
	purpleMax = (purpleHueMax, purpleSatMax, purpleValMax)

	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(hsv,purpleMin,purpleMax)

	masked_image = cv2.bitwise_and(image, image, mask=mask)


	# Show the frame
	# cv2.imshow("Frame", image)
	cv2.imshow("purple Frame", masked_image)

	key = cv2.waitKey(1) & 0xFF
	if key == 27 : # esc
		break
	elif  key == ord("q"):
		purpleHueMin = purpleHueMin - 5
		print(f"purpleHueMin : {purpleHueMin}")
	elif key == ord("e"):
		purpleHueMin = purpleHueMin + 5
		print(f"purpleHueMin : {purpleHueMin}")
	elif key == ord("a"): 
		purpleSatMin = purpleSatMin - 5
		print(f"purpleSatMin : {purpleSatMin}")
	elif key == ord("d"): 
		purpleSatMin = purpleSatMin + 5 
		print(f"purpleSatMin : {purpleSatMin}")
	elif key == ord("z"): 
		purpleValMin = purpleValMin - 5
		print(f"purpleValMin : {purpleValMin}")
	elif key == ord("c"): 
		purpleValMin = purpleValMin + 5
		print(f"purpleValMin : {purpleValMin}")
	elif  key == ord("t"):
		purpleHueMax = purpleHueMax - 5
		print(f"purpleHueMax : {purpleHueMax}")
	elif key == ord("u"):
		purpleHueMax = purpleHueMax + 5
		print(f"purpleHueMax : {purpleHueMax}")
	elif key == ord("g"): 
		purpleSatMax = purpleSatMax - 5
		print(f"purpleSatMax : {purpleSatMax}")
	elif key == ord("j"): 
		purpleSatMax = purpleSatMax + 5 
		print(f"purpleSatMax : {purpleSatMax}")
	elif key == ord("v"): 
		purpleValMax = purpleValMax - 5
		print(f"purpleValMax : {purpleValMax}")
	elif key == ord("n"): 
		purpleValMax = purpleValMax + 5
		print(f"purpleValMax : {purpleValMax}")
	elif key == ord("p"):
		print(f"purpleMin : ({purpleHueMin}, {purpleSatMin}, {purpleValMin})")
		print(f"purpleMax : ({purpleHueMax}, {purpleSatMax}, {purpleValMax})")



		
