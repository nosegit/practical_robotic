import zmq
import json
import numpy as np

class MapClient:
    def __init__(self, host="127.0.0.1", map_pub_port=5560, map_update_port=5561):
        self.context = zmq.Context()
        
        # 1. Socket for Sending Updates (Used by Robot Mapper)
        self.update_socket = self.context.socket(zmq.PUB)
        self.update_socket.connect(f"tcp://{host}:{map_update_port}")
        
        # 2. Socket for Receiving Map (Used by Navigator/Viewer)
        self.map_sub_socket = self.context.socket(zmq.SUB)
        self.map_sub_socket.connect(f"tcp://{host}:{map_pub_port}")
        self.map_sub_socket.subscribe("MAP")
        
        # Optimization: Only keep the last message in the internal buffer if possible
        # (Note: CONFLATE works on some systems, but the while-loop below is a universal fallback)
        try:
            self.map_sub_socket.setsockopt(zmq.CONFLATE, 1)
        except:
            self.map_sub_socket.setsockopt(zmq.LINGER, 0)

    def send_obstacle(self, x, y, confidence=5):
        """Notifies map server of obstacle."""
        msg = f"OBS {x:.3f} {y:.3f} {int(confidence)}"
        self.update_socket.send_string(msg)

    def get_latest_map(self):
        """
        Drains the socket queue and returns ONLY the absolute latest map.
        Prevents 'lag' caused by processing old buffered frames.
        """
        latest_parts = None
        
        # Loop to consume ALL waiting messages
        while True:
            try:
                parts = self.map_sub_socket.recv_multipart(flags=zmq.NOBLOCK)
                latest_parts = parts # Keep updating this variable
            except zmq.Again:
                # No more data in queue
                break
        
        # If we didn't receive anything, return None
        if latest_parts is None:
            return None
            
        # Process only the last message
        try:
            header = json.loads(latest_parts[1].decode('utf-8'))
            grid_data = np.frombuffer(latest_parts[2], dtype=np.int8)
            
            size = int(np.sqrt(len(grid_data)))
            grid = grid_data.reshape((size, size))
            return grid, header
        except Exception as e:
            print(f"Map decode error: {e}")
            return None

    def close(self):
        self.update_socket.close()
        self.map_sub_socket.close()
        self.context.term()