# Machine vision lab - Practical Robotics
# test_camera.py: Grab the camera framebuffer in OpenCV and display using cv2.imshow


# Import the necessary packages
import picamera2
import time
import cv2
import numpy as np

def noiseFilter(mask):
	kernel = np.ones((5,5), np.uint8)
	mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
	# mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

	return mask

# Initialise the camera and create a reference to it
camera = picamera2.Picamera2()
config = camera.create_preview_configuration(main={'format': 'RGB888', 'size' : (640,480)})
camera.configure(config)
camera.start()

# Allow the camera time to warm up
time.sleep(0.1)

# red masking in hse (hue[0,179], saturation[0,255], value)
lower_red1 = (0, 100, 100)
upper_red1 = (10, 255, 255)

lower_red2 = (170, 100, 100)
upper_red2 = (179, 255, 255) 

# green masking in hse (hue[0,179], saturation[0,255], value)
greenMin = (40, 100, 100)
greenMax = (80, 255, 255)

# blue masking in hse (hue[0,179], saturation[0,255], value)
blueMin = (80,190,20)
blueMax = (120,255,255)

# purple masking in hse (hue[0,179], saturation[0,255], value)
purpleMin = (130, 100, 100)
purpleMax = (165, 255, 255)

# Blob detection
blob_params = cv2.SimpleBlobDetector_Params()
blob_params.thresholdStep = 255
blob_params.minRepeatability = 1
blob_params.blobColor = 255
blob_params.filterByColor = False
blob_params.filterByArea = False
blob_params.filterByCircularity = False
blob_params.filterByInertia = False
blob_params.filterByConvexity = False
blob_detector = cv2.SimpleBlobDetector_create(blob_params)

# capture frames from the camera
while True:

	image = camera.capture_array("main")

	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

	# red have 2 hue range: 0-10 && 170-179
	mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
	mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
	mask_red = cv2.add(mask1, mask2)
	mask_red = noiseFilter(mask_red)

	mask_green = cv2.inRange(hsv,greenMin,greenMax)
	mask_green = noiseFilter(mask_green)

	mask_blue = cv2.inRange(hsv,blueMin,blueMax)
	mask_blue = noiseFilter(mask_blue)

	mask_purple = cv2.inRange(hsv,purpleMin,purpleMax)
	mask_purple = noiseFilter(mask_purple)

	keypoints = blob_detector.detect(mask_red)
	kp_image_red = cv2.drawKeypoints(mask_red, keypoints, None)
	# kp_image_red = cv2.drawKeypoints(image, keypoints, None)

	
	keypoints = blob_detector.detect(mask_green)
	kp_image_green = cv2.drawKeypoints(mask_green, keypoints, None)
	# kp_image_green = cv2.drawKeypoints(image, keypoints, None)
	
	keypoints = blob_detector.detect(mask_blue)
	kp_image_blue = cv2.drawKeypoints(mask_blue, keypoints, None)
	# kp_image_blue = cv2.drawKeypoints(image, keypoints, None)

	keypoints = blob_detector.detect(mask_purple)
	kp_image_purple = cv2.drawKeypoints(mask_purple, keypoints, None)
	# kp_image_purple = cv2.drawKeypoints(image, keypoints, None)

	# Add text to them to tell them apart
	cv2.putText(kp_image_red,	'RED',		(50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
	cv2.putText(kp_image_green,	'GREEN',	(50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
	cv2.putText(kp_image_blue,	'BLUE',		(50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
	cv2.putText(kp_image_purple,'PURPLE',	(50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

	top_row = np.hstack((kp_image_red,kp_image_green))
	bottom_row = np.hstack((kp_image_blue,kp_image_purple))
	combined_image = np.vstack((top_row, bottom_row))

	# Show the frame
	# cv2.imshow("Frame", image)
	cv2.imshow("Filter", combined_image)
	
	cv2.imshow("Camere",image)

	key = cv2.waitKey(1) & 0xFF
	# if the `q` key was pressed, break from the loop
	if key == ord("q"): 
		break


