# Machine vision lab - Practical Robotics
# test_camera.py: Grab the camera framebuffer in OpenCV and display using cv2.imshow


# Import the necessary packages
import picamera2
import time
import cv2

# Initialise the camera and create a reference to it
camera = picamera2.Picamera2()
config = camera.create_preview_configuration(main={'format': 'RGB888', 'size' : (640,480)})
camera.configure(config)
camera.start()

# Allow the camera time to warm up
time.sleep(0.1)

blueHueMin = 100
blueSatMin = 75
blueValMin = 75
# capture frames from the camera
while True:

	image = camera.capture_array("main")


	image_rotate = cv2.rotate(image,cv2.ROTATE_180)


	# blue masking in hse (hue[0,179], saturation[0,255], value)
	blueMin = (blueHueMin,blueSatMin,blueValMin)
	blueMax = (120,255,255)

	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(hsv,blueMin,blueMax)

	masked_image = cv2.bitwise_and(image, image, mask=mask)


	# Show the frame
	# cv2.imshow("Frame", image)
	cv2.imshow("Frame", masked_image)

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
