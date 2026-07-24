# Import the necessary packages
import picamera2
import time
import cv2
from cv2 import aruco

# Initialise the camera and create a reference to it
camera = picamera2.Picamera2()
config = camera.create_preview_configuration(main={'format': 'RGB888', 'size' : (640,480)})
camera.configure(config)
camera.start()

# Allow the camera time to warm up
time.sleep(0.1)

# Setup aruco dictionary and parameters
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
aruco_parameters = aruco.DetectorParameters_create()

# Create counter for FPS
frame_count = 0
start_time = time.time()

# capture frames from the camera
while True:
	image = camera.capture_array("main")
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	corners,ids,rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_parameters)
	
	frame_markers = aruco.drawDetectedMarkers(image, corners, ids)
   
	frame_count += 1
	average_fps = frame_count / ( time.time() - start_time )
	cv2.putText(frame_markers,"%2.1f fps" % average_fps, (50,50),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,255),2,cv2.LINE_AA)

	# Show the frame
	cv2.imshow("Frame", frame_markers)

	# if the `q` key was pressed, break from the loop
	if cv2.waitKey(1) & 0xFF == ord("q"):
		break
