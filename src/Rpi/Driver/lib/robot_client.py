import zmq
import json
import time

class RobotClient:
    def __init__(self, host="127.0.0.1", pub_port=5555, req_port=5556):
        self.context = zmq.Context()

        # 1. Command Socket (REQ)
        self.req_socket = self.context.socket(zmq.REQ)
        self.req_socket.connect(f"tcp://{host}:{req_port}")
        # IMPORTANT: Don't wait for unsent messages on exit
        self.req_socket.setsockopt(zmq.LINGER, 0)

        # 2. Telemetry Socket (SUB)
        self.sub_socket = self.context.socket(zmq.SUB)
        self.sub_socket.connect(f"tcp://{host}:{pub_port}")
        self.sub_socket.subscribe("") 
        self.sub_socket.setsockopt(zmq.LINGER, 0)

    def set_velocity(self, v, w):
        cmd = {"v": float(v), "w": float(w)}
        try:
            self.req_socket.send_json(cmd)
            return self.req_socket.recv_string()
        except zmq.ZMQError:
            return "ERROR"

    def stop(self):
        return self.set_velocity(0.0, 0.0)

    def get_state(self):
        latest_msg = None
        try:
            while True:
                latest_msg = self.sub_socket.recv_string(flags=zmq.NOBLOCK)
        except zmq.Again:
            pass

        if latest_msg:
            return json.loads(latest_msg)
        return None

    def close(self):
        """Safe cleanup to prevent hangs on Windows."""
        # Try to stop, but ignore errors if service is already down
        try:
            self.stop()
        except: 
            pass
            
        # Close sockets explicitly
        self.req_socket.close()
        self.sub_socket.close()
        
        # Terminate context
        self.context.term()