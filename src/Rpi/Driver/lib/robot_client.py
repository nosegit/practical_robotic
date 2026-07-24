import zmq
import json
import time
import math

class RobotClient:
    def __init__(self, host="127.0.0.1", pub_port=5555, req_port=5556):
        self.context = zmq.Context()

        # 1. Command Socket (REQ)
        self.req_socket = self.context.socket(zmq.REQ)
        self.req_socket.connect(f"tcp://{host}:{req_port}")
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
        try:
            self.stop()
        except: 
            pass
        self.req_socket.close()
        self.sub_socket.close()
        self.context.term()

    # --- HELPER FUNCTIONS ---

    def _wait_for_valid_state(self):
        state = None
        while state is None:
            state = self.get_state()
            if state is None:
                time.sleep(0.01)
        return state

    def move_distance(self, distance_meters, max_speed=0.2):
        """
        Moves specific distance using P-Control for precision.
        Slows down as it gets closer to target.
        """
        print(f"Moving {distance_meters:.2f}m...")
        start_state = self._wait_for_valid_state()
        start_x, start_y = start_state['x'], start_state['y']
        
        # P-Controller Gains
        Kp = 2.0   # Speed = Error * Kp
        MIN_SPEED = 0.01 # Minimum power to move motors
        TOLERANCE = 0.005 # 1cm accuracy

        target = abs(distance_meters)
        direction = 1.0 if distance_meters > 0 else -1.0

        while True:
            curr = self._wait_for_valid_state()
            dx = curr['x'] - start_x
            dy = curr['y'] - start_y
            traveled = math.sqrt(dx*dx + dy*dy)
            
            error = target - traveled
            
            if error <= TOLERANCE:
                break

            # Proportional Speed Calculation
            speed = error * Kp
            
            # Clamp Speed
            speed = min(speed, max_speed)
            speed = max(speed, MIN_SPEED)
            
            self.set_velocity(speed * direction, 0.0)
            time.sleep(0.02)

        self.stop()
        print(f"Move Complete. Error: {error:.4f}m")

    def rotate(self, angle_degrees, speed=0.5):
        print(f"Rotating {angle_degrees:.1f} degrees...")
        target_rad = math.radians(angle_degrees)
        start_state = self._wait_for_valid_state()
        last_theta = start_state['theta']
        accumulated_rotation = 0.0

        w = abs(speed) if angle_degrees > 0 else -abs(speed)
        self.set_velocity(0.0, w)

        while True:
            current_state = self._wait_for_valid_state()
            curr_theta = current_state['theta']
            delta = curr_theta - last_theta
            
            while delta > math.pi: delta -= 2 * math.pi
            while delta < -math.pi: delta += 2 * math.pi
            
            accumulated_rotation += delta
            last_theta = curr_theta

            if abs(accumulated_rotation) >= abs(target_rad):
                break
            time.sleep(0.05)

        self.stop()

    def rotate_to_angle(self, target_angle_degrees, speed=0.5):
        """
        Rotates the robot to face a specific absolute angle (e.g., 90 for North).
        Calculates the shortest path (Left or Right).
        """
        while True:
            # 1. Get current heading
            state = self._wait_for_valid_state()
            current_theta = state['theta'] # radians
            
            # 2. Calculate shortest turn
            target_rad = math.radians(target_angle_degrees)
            delta_rad = target_rad - current_theta
            
            # Normalize delta to [-pi, pi] so it turns the short way
            while delta_rad > math.pi: delta_rad -= 2 * math.pi
            while delta_rad < -math.pi: delta_rad += 2 * math.pi
            
            delta_deg = math.degrees(delta_rad)
            
            if abs(delta_deg) < 1.0:
                print(f"Already at target angle {target_angle_degrees}.")
                return

            print(f"Adjusting heading: Turning {delta_deg:.1f} degrees to reach {target_angle_degrees}.")
            
            # 3. Execute Turn using the existing relative rotate function
            self.rotate(delta_deg, speed)
            self.stop()
            time.sleep(0.1)
            
            speed = 0.1

    # --- NEW FUNCTION: CONTINUOUS WALL SWEEP ---

    def wall_sweep(self, sweep_arc=60, speed=0.3):
        """
        1. Rotates by 'turn_angle'.
        2. Continuous sweep of 'sweep_arc' degrees to find min IR.
        3. Rotates back until IR matches the found minimum.
        """
        print(f"Starting Continuous Wall Sweep...")
        
        # # 1. Initial Turn (e.g., 90 degrees) to face the general open direction
        # self.rotate(turn_angle)
        # time.sleep(0.2)

        # 2. Position at start of sweep (rotate left by half of arc)
        # We start "sweep_arc/2" to the right, so we can sweep left across the center
        start_offset = -(sweep_arc / 2)
        self.rotate(start_offset, speed=speed)
        time.sleep(0.2)
        
        # 3. Continuous Scan (Left to Right)
        print("Scanning (Continuous)...")
        min_ir_value = float('inf')
        
        start_state = self._wait_for_valid_state()
        start_theta = start_state['theta']
        target_rad = math.radians(sweep_arc)
        
        # Start rotating positive (Left)
        self.set_velocity(0.0, speed)
        
        while True:
            state = self._wait_for_valid_state()
            current_ir = state['ir']
            
            # Update Minimum
            if current_ir < min_ir_value:
                min_ir_value = current_ir
            
            # Check angle progress
            curr_theta = state['theta']
            delta = curr_theta - start_theta
            # Normalize delta
            while delta > math.pi: delta -= 2 * math.pi
            while delta < -math.pi: delta += 2 * math.pi
            
            if abs(delta) >= target_rad:
                break
            
            # No sleep here needed, we want high frequency sampling
            
        self.stop()
        print(f"Scan complete. Lowest IR found: {min_ir_value}")
        time.sleep(0.5)

        # 4. Rotate Back until IR matches Minimum
        print("Seeking minimum signal...")
        
        # Rotate opposite direction (Right)
        self.set_velocity(0.0, -speed)
        
        # Safety: Don't rotate back more than the arc + 20%
        safety_limit_rad = target_rad * 1.2
        start_return_state = self._wait_for_valid_state()
        start_return_theta = start_return_state['theta']
        
        while True:
            state = self._wait_for_valid_state()
            current_ir = state['ir']
            
            # Stop if we hit the minimum (with slight tolerance for noise)
            # We use a tolerance of +5 because sensor readings fluctuate
            if current_ir <= (min_ir_value + 0):
                print(f"Target found! IR: {current_ir}")
                self.rotate(angle_degrees=2, speed= 0.1)
                time.sleep(0.1)
                break
                
            # Safety Check: Stop if we turned too far without finding it
            curr_theta = state['theta']
            # Calculate how far we have turned back
            delta = curr_theta - start_return_theta
            while delta > math.pi: delta -= 2 * math.pi
            while delta < -math.pi: delta += 2 * math.pi
            
            if abs(delta) > safety_limit_rad:
                print("Warning: Could not re-acquire minimum IR. Stopping.")
                break
                
        self.stop()
        print("Wall Sweep Complete.")

    def set_theta(self, angle_degrees):
        angle_rad = math.radians(angle_degrees)

        # normalize
        while angle_rad > math.pi: angle_rad -= 2 * math.pi
        while angle_rad < -math.pi: angle_rad += 2 * math.pi

        # ad-hoc set rad with V = -31 , W = angle_rad
        self.set_velocity(-31, angle_rad)

    def get_snap_angle(self):
        angle_degrees = math.degrees(self._wait_for_valid_state()['theta'])
        return round(angle_degrees/90) * 90
    
    def reset_pose(self):

        for i in range(5):
            # ad-hoc reset pose with V = -30
            self.set_velocity(-30,0)
            
