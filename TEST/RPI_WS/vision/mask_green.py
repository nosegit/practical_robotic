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

greenHueMin = 40
greenSatMin = 100
greenValMin = 100
# capture frames from the camera
while True:

	image = camera.capture_array("main")


	image_rotate = cv2.rotate(image,cv2.ROTATE_180)


	# green masking in hse (hue[0,179], saturation[0,255], value)
	greenMin = (greenHueMin, greenSatMin, greenValMin)
	greenMax = (80, 255, 255)

	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(hsv,greenMin,greenMax)

	masked_image = cv2.bitwise_and(image, image, mask=mask)


	# Show the frame
	# cv2.imshow("Frame", image)
	cv2.imshow("Frame", masked_image)

	key = cv2.waitKey(1) & 0xFF
	if key == 27 : # esc
		break
	elif  key == ord("q"):
		greenHueMin = greenHueMin - 5
		print(f"greenHueMin : {greenHueMin}")
	elif key == ord("e"):
		greenHueMin = greenHueMin + 5
		print(f"greenHueMin : {greenHueMin}")
	elif key == ord("a"): 
		greenSatMin = greenSatMin - 5
		print(f"greenSatMin : {greenSatMin}")
	elif key == ord("d"): 
		greenSatMin = greenSatMin + 5 
		print(f"greenSatMin : {greenSatMin}")
	elif key == ord("z"): 
		greenValMin = greenValMin - 5
		print(f"greenValMin : {greenValMin}")
	elif key == ord("c"): 
		greenValMin = greenValMin + 5
		print(f"greenValMin : {greenValMin}")
