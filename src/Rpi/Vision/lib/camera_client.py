import zmq
import json
import time

class CameraClient:
    def __init__(self, host="127.0.0.1", port=5557):
        """
        Connects to the Vision Server.
        """
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(f"tcp://{host}:{port}")
        self.socket.subscribe("VISION")

        self.socket.setsockopt(zmq.LINGER, 0)
        
        # # Try to use CONFLATE (Server side filtering), but don't rely on it entirely.
        # try:
        #     self.socket.setsockopt(zmq.CONFLATE, 1)
        # except AttributeError:
        #     self.socket.setsockopt(zmq.LINGER, 0)

    def get_latest_data(self, timeout=None):
        """
        Drains the queue and returns ONLY the absolute latest vision payload.
        Args:
            timeout (int): Milliseconds to wait for at least one message. 
        """
        # 1. Wait for data (if timeout is set)
        if timeout:
            if self.socket.poll(timeout) == 0:
                return None
        
        latest_msg = None
        
        # 2. Loop to drain the entire queue (consume all buffered frames)
        while True:
            try:
                msg = self.socket.recv_string(flags=zmq.NOBLOCK)
                latest_msg = msg # Update variable with newer message
            except zmq.Again:
                # Queue is empty, we have the latest one
                break
        
        # 3. If we didn't receive anything, return None
        if latest_msg is None:
            return None
            
        try:
            # Parse the last message we got
            json_str = latest_msg[7:] # Remove topic "VISION "
            return json.loads(json_str)
        except Exception as e:
            print(f"Vision Client Error: {e}")
            return None

    def get_marker_pose(self, marker_id):
        """
        Finds a specific ArUco marker and returns its 3D pose data.
        Returns None if the marker is not currently visible.
        """
        data = self.get_latest_data(timeout=300)
        if not data or 'aruco' not in data:
            return None
            
        for marker in data['aruco']:
            if marker['id'] == marker_id:
                return marker
        return None
    
    def get_all_marker(self):
        arr = []
        for i in range(1,6):
            data = self.get_marker_pose(marker_id=i)
            if (data):
                arr.append(data)
    
        return arr if arr != [] else None

    def get_closest_shape(self, color=None, label=None):
        """
        Returns the largest/closest shape matching the criteria.
        """
        data = self.get_latest_data(timeout=100)
        if not data or 'shapes' not in data:
            return None
            
        best_match = None
        max_area = 0
        
        for shape in data['shapes']:
            # Filter logic
            if color and shape['color'] != color: continue
            if label and shape['label'] != label: continue
            
            # Find largest (assuming largest is closest)
            if shape['area'] > max_area:
                max_area = shape['area']
                best_match = shape
                
        return best_match
    
    def is_color(self, color=None):
        """
        Returns if there is object(shape) matching  the color.
        """
        data = self.get_latest_data(timeout=100)
        if not data or 'shapes' not in data:
            return None
        
        for shape in data['shapes']:
            # Filter logic
            if color and shape['color'] == color: 
                return True

        return False

# --- TEST USAGE ---
if __name__ == "__main__":
    client = CameraClient()
    print("Camera Client Started. Waiting for data...")
    
    while True:
        # # 1. Check for ArUco ID 1
        # marker = client.get_marker_pose(marker_id=1)
        # if marker:
        #     print(f"[ARUCO] ID: 1 | Dist: {marker['distance']:.2f}m | Yaw: {marker['yaw']:.1f}°")
        
        # # 2. Check for Red Triangle
        # shape = client.get_closest_shape(color="RED", label="Triangle")
        # if shape:
        #     print(f"[SHAPE] Found Red Triangle at {shape['center']}")

        dist_nearest_tag = 999
        markers = client.get_all_marker()
        if markers:
            for i in markers:
                print(i['yaw'])
                dist_nearest_tag = min(dist_nearest_tag,i['distance'])

        print(dist_nearest_tag)

        time.sleep(0.1)