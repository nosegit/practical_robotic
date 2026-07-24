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

redHueMin = 0
redSatMin = 100
redValMin = 75

redHueMax = 25

# capture frames from the camera
while True:

	image = camera.capture_array("main")



	# red masking in hse (hue[0,179], saturation[0,255], value)
	lower_red1 = (0, 100, 100)
	upper_red1 = (10, 255, 255)

	lower_red2 = (170, 100, 100)
	upper_red2 = (179, 255, 255) 

	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
	mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
	final_mask = cv2.add(mask1, mask2)

	masked_image = cv2.bitwise_and(image, image, mask=final_mask)


	# Show the frame
	# cv2.imshow("Frame", image)
	cv2.imshow("Frame", masked_image)

	key = cv2.waitKey(1) & 0xFF
	if key == 27 : # esc
		break
	elif  key == ord("q"):
		redHueMin = redHueMin - 5
		print(f"redHueMin : {redHueMin}")
	elif key == ord("e"):
		redHueMin = redHueMin + 5
		print(f"redHueMin : {redHueMin}")
	elif key == ord("a"): 
		redSatMin = redSatMin - 5
		print(f"redSatMin : {redSatMin}")
	elif key == ord("d"): 
		redSatMin = redSatMin + 5 
		print(f"redSatMin : {redSatMin}")
	elif key == ord("z"): 
		redValMin = redValMin - 5
		print(f"redValMin : {redValMin}")
	elif key == ord("c"): 
		redValMin = redValMin + 5
		print(f"redValMin : {redValMin}")
	elif key == ord("r"): 
		redHueMax = redHueMax - 5
		print(f"redHueMax : {redHueMax}")
	elif key == ord("y"): 
		redHueMax = redHueMax + 5
		print(f"redHueMax : {redHueMax}")

