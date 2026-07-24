import zmq
import json
import time

class RobotClient:
    def __init__(self, host="127.0.0.1", pub_port=5555, req_port=5556):
        """
        Initializes the connection to the Robot Service.
        """
        self.context = zmq.Context()

        # 1. Command Socket (Request/Reply)
        # We send commands here and wait for an "ACK"
        self.req_socket = self.context.socket(zmq.REQ)
        self.req_socket.connect(f"tcp://{host}:{req_port}")

        # 2. Telemetry Socket (Subscriber)
        # We listen for updates here
        self.sub_socket = self.context.socket(zmq.SUB)
        self.sub_socket.connect(f"tcp://{host}:{pub_port}")
        self.sub_socket.subscribe("") 
        # CONFLATE=1 ensures we always get the *newest* message, skipping old ones in the buffer
        self.sub_socket.setsockopt(zmq.CONFLATE, 1)

    def set_velocity(self, v, w):
        """
        Send a velocity command.
        v: Linear velocity (m/s)
        w: Angular velocity (rad/s)
        """
        cmd = {"v": float(v), "w": float(w)}
        self.req_socket.send_json(cmd)
        # We must receive the reply to complete the REQ/REP cycle
        return self.req_socket.recv_string()

    def stop(self):
        """Helper to stop the robot immediately."""
        return self.set_velocity(0.0, 0.0)

    def get_state(self):
        """
        Returns the latest state dict, or None if no new data is available.
        Non-blocking.
        """
        latest_msg = None
        try:
            # Loop continuously until the queue is empty (zmq.Again raised)
            while True:
                latest_msg = self.sub_socket.recv_string(flags=zmq.NOBLOCK)
        except zmq.Again:
            # Queue is empty, we have the latest message in 'latest_msg'
            pass

        if latest_msg:
            return json.loads(latest_msg)

    def close(self):
        """Clean up sockets."""
        self.stop()
        self.context.term()