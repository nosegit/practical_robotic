import zmq
import numpy as np
import time
import json
import threading

# --- CONFIGURATION ---
MAP_PUB_PORT = 5560    # Port to publish complete map data
MAP_UPDATE_PORT = 5561 # Port to receive sensor updates
MAP_SIZE_METERS = 5.0
RESOLUTION = 0.04      # 4cm per cell
GRID_CELLS = int(MAP_SIZE_METERS / RESOLUTION)

class MapServer:
    def __init__(self):
        self.context = zmq.Context()
        
        # 1. Map Publisher (PUB)
        # Publishes the entire grid array to navigators/viewers
        self.pub_socket = self.context.socket(zmq.PUB)
        self.pub_socket.bind(f"tcp://*:{MAP_PUB_PORT}")
        
        # 2. Update Listener (SUB)
        # Listens for "Here is a wall" messages from the robot mapper
        self.sub_socket = self.context.socket(zmq.SUB)
        self.sub_socket.bind(f"tcp://*:{MAP_UPDATE_PORT}")
        self.sub_socket.subscribe("OBS") # Subscribe to obstacle updates
        
        # 3. The Map Data (Occupancy Grid)
        # 0 = Unknown, 1 = Free, 100 = Occupied
        self.grid = np.zeros((GRID_CELLS, GRID_CELLS), dtype=int)
        self.center_idx = GRID_CELLS // 2
        
        self.running = True

    def world_to_grid(self, wx, wy):
        gx = int((wx / RESOLUTION) + self.center_idx)
        gy = int((wy / RESOLUTION) + self.center_idx)
        return gx, gy

    def update_loop(self):
        """Listens for obstacle hits and updates grid."""
        print(f"Map Server running. Pub: {MAP_PUB_PORT}, Sub: {MAP_UPDATE_PORT}")
        
        while self.running:
            try:
                # Receive update: "OBS x_meter y_meter value"
                msg = self.sub_socket.recv_string(flags=zmq.NOBLOCK)
                parts = msg.split()
                
                if parts[0] == "OBS":
                    wx = float(parts[1])
                    wy = float(parts[2])
                    val = int(parts[3]) # e.g., +5 for hit, -1 for free
                    
                    gx, gy = self.world_to_grid(wx, wy)
                    
                    # Boundary Check
                    if 0 <= gx < GRID_CELLS and 0 <= gy < GRID_CELLS:
                        self.grid[gy, gx] += val
                        # Clamp values (0 to 100)
                        self.grid[gy, gx] = max(0, min(100, self.grid[gy, gx]))
                        
            except zmq.Again:
                time.sleep(0.001)

    def publish_loop(self):
        """Periodically sends the full map."""
        while self.running:
            # Prepare metadata
            meta = {
                "size": MAP_SIZE_METERS,
                "res": RESOLUTION,
                "origin_idx": self.center_idx
            }
            
            # Serialize: Header JSON + Separator + Raw Bytes
            # This is efficient for large arrays
            header = json.dumps(meta).encode('utf-8')
            data = self.grid.astype(np.int8).tobytes()
            
            self.pub_socket.send_multipart([b"MAP", header, data])
            
            time.sleep(0.1) # 5Hz Map Publish Rate

    def start(self):
        t1 = threading.Thread(target=self.update_loop)
        t2 = threading.Thread(target=self.publish_loop)
        t1.start()
        t2.start()
        
        self.printInfo()

        try:
            while True: time.sleep(1)
        except KeyboardInterrupt:
            self.running = False
            t1.join()
            t2.join()

    def printInfo(self):
        print(f"Map Server Info:")
        print(f"  Map Size: {MAP_SIZE_METERS} meters")
        print(f"  Resolution: {RESOLUTION} meters/cell")
        print(f"  Grid Cells: {GRID_CELLS} x {GRID_CELLS}")

if __name__ == "__main__":
    server = MapServer()
    server.start()