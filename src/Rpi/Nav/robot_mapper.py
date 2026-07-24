import sys
import os
import time
import math
import numpy as np

# Add parent directory to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from Driver.lib.robot_client import RobotClient
from Nav.lib.map_client import MapClient

# --- SENSOR CONFIGURATION ---

# 1. Front IR Sensor (Points Forward / 0 deg)
IR_OFFSET_X = 0.15   # 0.15m (15cm) forward from center
IR_OFFSET_Y = 0.025
IR_MIN_CM = 3
IR_MAX_CM = 20

# 2. Left Ultrasonic Sensor (Points Left / +90 deg)
US_L_OFFSET_X = 0.055 
US_L_OFFSET_Y = 0.08 
US_L_MIN_CM = 6
US_L_MAX_CM = 60

# 3. Right Ultrasonic Sensor (Points Right / -90 deg)
US_R_OFFSET_X = 0.055
US_R_OFFSET_Y = -0.08 
US_R_MIN_CM = 6
US_R_MAX_CM = 60

# --- RAY CASTING CONFIG ---
RAY_STEP_SIZE = 0.04  # Step size for clearing (meters). Match your map resolution.
CLEAR_CONFIDENCE = -2 # How much to 'subtract' from grid for free space
HIT_CONFIDENCE = 5    # How much to 'add' to grid for obstacles

def process_sensor_reading(state, raw_dist_cm, mount_x, mount_y, mount_angle_deg, min_cm, max_cm, map_client):
    """
    1. Calculates Sensor World Position.
    2. Calculates Obstacle World Position.
    3. Clears the line between them (Ray Casting).
    4. Marks the obstacle.
    """
    if min_cm < raw_dist_cm < max_cm:
        dist_m = raw_dist_cm / 100.0
        
        # --- A. GEOMETRY CALCULATIONS ---
        theta = state['theta']
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        
        # 1. Calculate World Position of the SENSOR itself
        # Rotate mount offset by robot heading + add robot pos
        sens_world_x = state['x'] + (mount_x * cos_t - mount_y * sin_t)
        sens_world_y = state['y'] + (mount_x * sin_t + mount_y * cos_t)

        # 2. Calculate World Position of the OBSTACLE
        # Vector from Sensor to Obstacle is 'dist_m' long, rotated by (Theta + MountAngle)
        sens_global_angle = theta + math.radians(mount_angle_deg)
        
        obs_x = sens_world_x + (dist_m * math.cos(sens_global_angle))
        obs_y = sens_world_y + (dist_m * math.sin(sens_global_angle))
        
        # --- B. NOISE REMOVAL (RAY CLEARING) ---
        # "Anything in front of it set to zero"
        
        # Calculate total distance and number of steps
        dx = obs_x - sens_world_x
        dy = obs_y - sens_world_y
        # total_dist = math.sqrt(dx*dx + dy*dy)
        # steps = int(total_dist / RAY_STEP_SIZE)
        
        # # Iterate along the ray
        # for i in range(steps):
        #     # Interpolate position (0.0 to 1.0)
        #     # We skip the very last step to avoid clearing the wall itself
        #     alpha = i / steps
            
        #     clear_x = sens_world_x + (dx * alpha)
        #     clear_y = sens_world_y + (dy * alpha)
            
        #     # Send "Free Space" update (Negative Value)
        #     map_client.send_obstacle(clear_x, clear_y, confidence=CLEAR_CONFIDENCE)

        # --- C. MARK OBSTACLE ---
        # Send "Occupied" update (Positive Value)
        map_client.send_obstacle(obs_x, obs_y, confidence=HIT_CONFIDENCE)

def main():
    robot = RobotClient(host="127.0.0.1")
    map_client = MapClient(host="127.0.0.1")
    
    print("Mapper running with Noise Removal (Ray Casting)...")

    try:
        while True:
            state = robot.get_state()
            if state is None: 
                time.sleep(0.005)
                continue

            # 1. Front IR
            if 'ir' in state:
                process_sensor_reading(state, state['ir'], 
                                     IR_OFFSET_X, IR_OFFSET_Y, 0, 
                                     IR_MIN_CM, IR_MAX_CM, map_client)

            # 2. Left US
            if 'us_l' in state:
                process_sensor_reading(state, state['us_l'], 
                                     US_L_OFFSET_X, US_L_OFFSET_Y, 90, 
                                     US_L_MIN_CM, US_L_MAX_CM, map_client)

            # 3. Right US
            if 'us_r' in state:
                process_sensor_reading(state, state['us_r'], 
                                     US_R_OFFSET_X, US_R_OFFSET_Y, -90, 
                                     US_R_MIN_CM, US_R_MAX_CM, map_client)
                
            time.sleep(0.02)

    except KeyboardInterrupt:
        pass
    finally:
        robot.close()
        map_client.close()

if __name__ == "__main__":
    main()