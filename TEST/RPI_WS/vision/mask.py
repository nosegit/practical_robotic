# Machine vision lab - Practical Robotics
# test_camera.py: Grab the camera framebuffer in OpenCV and display using cv2.imshow


# Import the necessary packages
import picamera2
import time
import cv2
import numpy as np

# Initialise the camera and create a reference to it
camera = picamera2.Picamera2()
config = camera.create_preview_configuration(main={'format': 'RGB888', 'size' : (640,480)})
camera.configure(config)
camera.start()

# Allow the camera time to warm up
time.sleep(0.1)

# red masking in hse (hue[0,179], saturation[0,255], value)
redMin = (0, 100, 75)
redMax = (25, 255, 255)

# green masking in hse (hue[0,179], saturation[0,255], value)
greenMin = (40, 50, 50)
greenMax = (80, 255, 255)

# blue masking in hse (hue[0,179], saturation[0,255], value)
blueMin = (100,75,75)
blueMax = (120,255,255)

# purple masking in hse (hue[0,179], saturation[0,255], value)
purpleMin = (130, 50, 50)
purpleMax = (160, 255, 255)

start_time = time.time()
frame_count = 0
# capture frames from the camera
while True:

	image = camera.capture_array("main")

	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	mask_red = cv2.inRange(hsv,redMin,redMax)
	mask_green = cv2.inRange(hsv,greenMin,greenMax)
	mask_blue = cv2.inRange(hsv,blueMin,blueMax)
	mask_purple = cv2.inRange(hsv,purpleMin,purpleMax)


	masked_red_image = cv2.bitwise_and(image, image, mask=mask_red)
	masked_green_image = cv2.bitwise_and(image, image, mask=mask_green)
	masked_blue_image = cv2.bitwise_and(image, image, mask=mask_blue)
	masked_purple_image = cv2.bitwise_and(image, image, mask=mask_purple)


	# Add text to them to tell them apart
	cv2.putText(masked_red_image,	'RED',		(50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
	cv2.putText(masked_green_image,	'GREEN',	(50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
	cv2.putText(masked_blue_image,	'BLUE',		(50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
	cv2.putText(masked_purple_image,'PURPLE',	(50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

	top_row = np.hstack((masked_red_image,masked_green_image))
	bottom_row = np.hstack((masked_blue_image,masked_purple_image))
	combined_image = np.vstack((top_row, bottom_row))

	frame_count += 1
	average_fps = frame_count / ( time.time() - start_time )
	cv2.putText(combined_image,"%2.1f fps" % average_fps, (50,50),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,255),2,cv2.LINE_AA)

	# Show the frame
	# cv2.imshow("Frame", image)
	cv2.imshow("Frame", combined_image)

	# if the `q` key was pressed, break from the loop
	if cv2.waitKey(1) & 0xFF == ord("q"):
		break
