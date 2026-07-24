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


	image_rotate = cv2.rotate(image,cv2.ROTATE_180)

	# blue masking in hse (hue[0,179], saturation[0,255], value)
	blueMin = (100,75,75)
	blueMax = (130,255,255)

	hsv = cv2.cvtColor(image_rotate, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(hsv,blueMin,blueMax)

	keypoints = blob_detector.detect(mask)
	kp_image = cv2.drawKeypoints(mask,keypoints,None)

	# Show the frame
	# cv2.imshow("Frame", image)
	cv2.imshow("Frame", kp_image)

	# if the `q` key was pressed, break from the loop
	if cv2.waitKey(1) & 0xFF == ord("q"):
		break
