import sys
import os
import time
import numpy as np
import math

# --- Import Robot Client ---
# Modified to look for Driver.lib structure based on your setup
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../')))
try:
    from Driver.lib.robot_client import RobotClient
except ImportError:
    try:
        from lib.robot_client import RobotClient
    except ImportError:
        from robot_client import RobotClient


# --- CONFIGURATION CONSTANTS ---

# Target Speed
LINEAR_SPEED_M_S = 0.05      # Max linear velocity for P-controller correction (m/s)
LINEAR_SPEED_M_S_STRAIGHT = 0.10 # Constant speed for deliberate forward moves (e.g., after a turn)
TURN_SPEED_RAD_S = 0.5 # Fixed angular speed for 90 degree turns (0.5 radians per second)

# Wall Following Parameters (in CM)
WALL_FOLLOW_DISTANCE_CM = 6 # Target distance from the right wall
WALL_FOLLOW_TOLERANCE_CM = 3 # Error tolerance

# Obstacle Thresholds (in CM)
FRONT_CLEAR_THRESHOLD_CM = 10 # Distance required for the front to be considered clear
FRONT_BLOCKED_THRESHOLD_CM = WALL_FOLLOW_DISTANCE_CM-3 # Anything closer is considered a solid block

# Distance to travel after a turn before re-evaluating (in METERS)
STEP_DISTANCE_M = 0.05 
ANGLE_TOLERANCE_RAD = np.radians(3.0) # Tolerance for rotation (3 degrees)
DISTANCE_TOLERANCE_M = 0.01 # Tolerance for linear distance (1 cm)

# P-Controller Gain for Straight Path Correction
P_GAIN = 0.10 
# Maximum correction angular velocity
MAX_W = np.radians(20) 

def normalize_angle(angle):
    """Wraps an angle to the range (-pi, pi]"""
    return math.atan2(math.sin(angle), math.cos(angle))


def rotate_angle(robot: RobotClient, angle_rad: float):
    """
    Rotates the robot by a specified angle (positive is counter-clockwise) 
    by subscribing to the odometry state until the target angle is reached.
    """
    if abs(angle_rad) < ANGLE_TOLERANCE_RAD:
        return

    # Get initial theta for reference
    initial_state = robot.get_state()
    if not initial_state: return
    
    # Use the raw theta from the state
    initial_theta = initial_state['theta']
    target_theta = initial_theta + angle_rad
    
    target_theta = normalize_angle(target_theta)
    
    # Determine direction based on angle sign (Same strategy as odometry_validator.py)
    if angle_rad > 0:
        robot.set_velocity(0.0, TURN_SPEED_RAD_S) # CCW (0.5 rad/s)
    else:
        robot.set_velocity(0.0, -TURN_SPEED_RAD_S) # CW (-0.5 rad/s)
    
    print(f"-> Starting rotation. Target change: {np.degrees(angle_rad):.1f} deg.")

    start_time = time.time()
    MAX_TURN_TIME = 5.0 # Failsafe
    
    while time.time() - start_time < MAX_TURN_TIME:
        state = robot.get_state()
        if state is None:
            time.sleep(0.01)
            continue

        current_theta = state['theta']
        
        # Calculate angular difference, handling the wrap-around at +/- PI
        diff = normalize_angle(target_theta - current_theta)
        
        # Check if the remaining difference is within tolerance.
        if abs(diff) < ANGLE_TOLERANCE_RAD:
            print(f"<- Rotation complete. Final Theta: {np.degrees(current_theta):.1f} deg.")
            break
        
        time.sleep(0.01)

    robot.stop()
    time.sleep(0.5)


def move_distance(robot: RobotClient, distance_m: float, v_target: float):
    """
    Moves the robot forward by a specified distance (in meters) 
    by subscribing to the odometry state until the target distance is reached.
    """
    if distance_m <= DISTANCE_TOLERANCE_M:
        return
        
    initial_state = robot._wait_for_valid_state()
    if not initial_state: return
    
    initial_x = initial_state['x']
    initial_y = initial_state['y']
    
    print(f"-> Starting forward move. Target distance: {distance_m:.2f}m.")
    
    robot.set_velocity(v_target, 0.0)

    start_time = time.time()
    MAX_MOVE_TIME = 10.0 # Failsafe
    
    while time.time() - start_time < MAX_MOVE_TIME:
        state = robot.get_state()
        if state is None:
            time.sleep(0.01)
            continue
            
        current_x = state['x']
        current_y = state['y']
        
        # Calculate distance traveled using the Euclidean distance formula
        distance_traveled = np.sqrt((current_x - initial_x)**2 + (current_y - initial_y)**2)
        
        if distance_traveled >= distance_m:
            print(f"<- Movement complete. Traveled: {distance_traveled:.2f}m.")
            break
            
        time.sleep(0.01)

    robot.stop()
    time.sleep(0.5)


def follow_wall_step(robot: RobotClient, state: dict):
    """
    Implements one step of the Right-Hand Wall Following algorithm.
    """
    ir = state['ir']
    us_r = state['us_r']

    # 1. Decision Logic (Right-Hand Rule)
    
    # Check if the right side is OPEN 
    right_is_open = us_r > (WALL_FOLLOW_DISTANCE_CM + WALL_FOLLOW_TOLERANCE_CM * 2)
    
    # Check if the front is clear
    front_is_clear = ir > FRONT_CLEAR_THRESHOLD_CM

    # Check for a complete block in front
    front_is_blocked = ir < FRONT_BLOCKED_THRESHOLD_CM
    
    v = LINEAR_SPEED_M_S # Linear velocity for P-controller correction
    w = 0.0              # Angular velocity

    # --- THE WALL FOLLOWING HIERARCHY ---
    
    if front_is_blocked:
        # A. Dead End: Turn 90 degrees Left 
        print(f"--- DEAD END/BLOCK --- Turning 90 degrees Left. IR: {ir:.1f}cm")
        rotate_angle(robot, np.radians(90))
        if(robot._wait_for_valid_state()['ir'] < 20) : 
            robot.wall_sweep(sweep_arc=20, speed = 0.1)
            robot.set_theta(angle_degrees=robot.get_snap_angle())
        return

    elif right_is_open:
        # B. Right Turn Opportunity: Turn right and move forward to clear the corner
        robot.stop()
        print(f"--- OPEN RIGHT --- Turning 90 degrees Right. US_R: {us_r:.1f}cm")
        move_distance(robot, 0.15, LINEAR_SPEED_M_S_STRAIGHT)
        rotate_angle(robot, np.radians(-90))
        if(robot._wait_for_valid_state()['ir'] < 20) : 
            robot.wall_sweep(sweep_arc=20, speed = 0.1)
            robot.set_theta(angle_degrees=robot.get_snap_angle())
        move_distance(robot, 0.20, LINEAR_SPEED_M_S_STRAIGHT)
        return
    
    elif not front_is_clear:
        # C. Blocked Front (Corner): Pivot left and move forward to clear the corner
        robot.stop() 
        print(f"--- CORNER/FRONT BLOCKED --- Pivoting 90 degrees Left. IR: {ir:.1f}cm")
        rotate_angle(robot, np.radians(90))
        if(robot._wait_for_valid_state()['ir'] < 20) : 
            robot.wall_sweep(sweep_arc=20, speed = 0.1)
            robot.set_theta(angle_degrees=robot.get_snap_angle())
        move_distance(robot, STEP_DISTANCE_M, LINEAR_SPEED_M_S_STRAIGHT)
        return
    
    else: 
        # D. Straight Path (Cruising): P-controller for wall alignment
        
        error = us_r - WALL_FOLLOW_DISTANCE_CM
        
        # Proportional controller: turn right if too far (negative w), turn left if too close (positive w)
        w = -P_GAIN * error 

        # Limit angular speed
        w = max(-MAX_W, min(MAX_W, w))

        print(f"-> STRAIGHT. US_R: {us_r:.1f}cm. Steering W: {w:.3f}")

        # Execute the determined velocity
        robot.set_velocity(v, w)

def main():
    print("Connecting to Robot Service...")
    robot = RobotClient(host="127.0.0.1")
    
    if not robot._wait_for_valid_state():
        print("❌ ERROR: Robot Service is NOT running!")
        print("   -> Run 'python robot_service.py' first.")
        robot.close()
        return
    
    robot.stop()
    time.sleep(1.0)
    
    print("Starting Wall Follower Maze Exploration (Odometry Controlled). Press Ctrl+C to stop.")
    
    try:
        while True:
            # 1. Get the latest state
            state = robot.get_state()
            if state is None:
                time.sleep(0.01)
                continue
            
            # Unit Check: Assuming 'ir', 'us_l', and 'us_r' are already in Centimeters (CM) 
            # as confirmed by the user. No unit conversion is performed here.
            
            # 2. Execute one step of the exploration logic
            # This function blocks for turns and distance moves (A, B, C)
            # or continuously controls speed for straight paths (D).
            follow_wall_step(robot, state)
            
            # Short pause for the continuous straight path control loop
            time.sleep(0.05) 

    except KeyboardInterrupt:
        print("\nExploration Stopped by user.")
        
    finally:
        robot.stop()
        print("Robot stopped.")
        robot.close()

if __name__ == "__main__":
    main()
