"""
CameraViewer: Real-time JPEG video stream viewer for robot camera feed via ZMQ.
Subscribes to CameraServer on port 5558, drains buffer for latest frame, and displays in OpenCV.

Features:
  - Low-latency frame draining (consumes all buffered frames to get newest)
  - Efficient JPEG decoding with zero-copy numpy buffer conversion
  - Optional ZMQ CONFLATE for socket-level frame buffering
  - Graceful shutdown on 'q' key or Ctrl+C

Configuration: Set SERVER_IP to Raspberry Pi address (default: localhost)
"""
import zmq
import numpy as np
import cv2

# --- CONFIGURATION ---
SERVER_IP = "localhost" # CHANGE THIS to Pi's IP address
VIDEO_PORT = 5558       # Must match camera_server.py

def main():
    """
    Main viewer loop: connect to video stream, drain buffer for latest frame, decode and display.
    Handles graceful shutdown on 'q' key or Ctrl+C.
    """
    print(f"Connecting to Video Stream at {SERVER_IP}:{VIDEO_PORT}...")
    
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect(f"tcp://{SERVER_IP}:{VIDEO_PORT}")
    socket.subscribe("") # Subscribe to everything
    
    # Enable Conflate (Only keep the latest frame in buffer)
    # This ensures low latency at the socket level.
    try:
        socket.setsockopt(zmq.CONFLATE, 1)
    except AttributeError:
        # If the system ZMQ doesn't support conflate, we rely on the manual loop below
        socket.setsockopt(zmq.LINGER, 0)
        print("Warning: ZMQ Conflate not supported, latency might increase.")

    window_name = "Robot Vision Stream"
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)

    try:
        while True:
            # 1. Drain the ZMQ buffer to get the absolute latest frame
            # We loop recv(NOBLOCK) until the queue is empty (zmq.Again)
            jpg_bytes = None
            try:
                while True:
                    # Keep reading and overwriting 'jpg_bytes' until we hit the exception
                    msg = socket.recv(flags=zmq.NOBLOCK)
                    jpg_bytes = msg
            except zmq.Again:
                pass # Queue is empty, we have the latest message in 'jpg_bytes'

            # 2. Decode only if we received data
            if jpg_bytes is not None:
                # np.frombuffer is very fast (zero-copy)
                img_np = np.frombuffer(jpg_bytes, dtype=np.uint8)
                frame = cv2.imdecode(img_np, cv2.IMREAD_COLOR)
                
                if frame is not None:
                    cv2.imshow(window_name, frame)
                else:
                    print("Error decoding frame")
            
            # 3. Handle User Input
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

    except KeyboardInterrupt:
        print("Stopping Viewer...")
    finally:
        socket.close()
        context.term()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()