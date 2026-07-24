import sys
import os
import time
import math

# Use the specific import path the user provided
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../')))
try:
    from Driver.lib.robot_client import RobotClient
except ImportError:
    try:
        from lib.robot_client import RobotClient
    except ImportError:
        from robot_client import RobotClient


# --- CONFIGURATION ---
LINEAR_SPEED = 0.2    # meters/second
ANGULAR_SPEED = 0.5   # radians/second (about 28 degrees/sec)
SIDE_LENGTH = 0.5     # meters for each side of the square
TARGET_ANGLES = [math.pi / 2, math.pi, -math.pi / 2, 0] # 90, 180, -90, 0 degrees in radians
ANGLE_TOLERANCE = 0.05/3 # radians (about 3 degrees)

def normalize_angle(angle):
    """
    Wraps an angle to the range (-pi, pi]
    This is necessary because theta resets from pi to -pi (180 to -180 degrees).
    """
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle <= -math.pi:
        angle += 2 * math.pi
    return angle

def go_to_angle(robot, target_theta, current_state):
    """
    Spins the robot in place until the target theta is reached.
    """
    start_time = time.time()
    current_theta = current_state['theta']
    target_theta = normalize_angle(target_theta)

    # Calculate the shortest turn direction (clockwise or counter-clockwise)
    error = normalize_angle(target_theta - current_theta)

    if abs(error) < ANGLE_TOLERANCE:
        print(f"Already within tolerance ({error:.3f} rad).")
        return

    # Determine direction based on error sign
    if error > 0:
        robot.set_velocity(0.0, ANGULAR_SPEED) # Spin Counter-Clockwise (positive w)
        direction = "CCW"
    else:
        robot.set_velocity(0.0, -ANGULAR_SPEED) # Spin Clockwise (negative w)
        direction = "CW"

    print(f"   [TURN] Target: {math.degrees(target_theta):.1f}° | Start: {math.degrees(current_theta):.1f}° | Turning {direction}...")

    # Loop until the target is hit or timeout (for safety)
    while abs(error) > ANGLE_TOLERANCE and (time.time() - start_time) < 15: # 15s timeout
        state = robot.get_state()
        if state:
            current_theta = state['theta']
            error = normalize_angle(target_theta - current_theta)
            print(f"\r   [ODOM] Current: {math.degrees(current_theta):.1f}° | Error: {math.degrees(error):.1f}°", end="")
        time.sleep(0.02) # Fast loop for precise control

    robot.stop()
    print("\n   [TURN] Complete.")


def go_forward(robot, distance, current_state):
    """
    Drives the robot straight until the target distance is reached.
    """
    start_x = current_state['x']
    start_y = current_state['y']
    
    print(f"   [DRIVE] Target distance: {distance:.2f} m.")
    robot.set_velocity(LINEAR_SPEED, 0.0)

    start_time = time.time()
    
    while True:
        state = robot.get_state()
        if state is None:
            time.sleep(0.02)
            continue
            
        # Calculate distance traveled using Pythagorean theorem
        dx = state['x'] - start_x
        dy = state['y'] - start_y
        dist_traveled = math.sqrt(dx**2 + dy**2)
        
        print(f"\r   [ODOM] Traveled: {dist_traveled:.3f} m / {distance:.2f} m", end="")
        
        if dist_traveled >= distance or (time.time() - start_time) > 15: # 15s timeout
            break
            
        time.sleep(0.02)

    robot.stop()
    print("\n   [DRIVE] Complete.")
    

def main():
    print("Initializing Client...")
    robot = RobotClient(host="127.0.0.1")

    if not robot.wait_for_connection():
        print("❌ ERROR: Robot Service is NOT running!")
        print("   -> Run 'python robot_service.py' first.")
        robot.close()
        return

    print("✅ Connected! Starting Odometry Validation (Square Path).")
    
    # 1. Wait for initial state
    current_state = robot.get_state()
    while current_state is None:
        current_state = robot.get_state()
        time.sleep(0.1)

    print("-" * 50)
    print(f"Initial State: X:{current_state['x']:.3f} Y:{current_state['y']:.3f} Theta:{math.degrees(current_state['theta']):.1f}°")
    print("-" * 50)

    try:
        for j in range(10):
            # Loop 4 times to complete a square
            for i, target_theta in enumerate(TARGET_ANGLES):
                print(f"\n--- SIDE {i+1} ---")
                
                # # --- STEP 1: DRIVE ---
                # go_forward(robot, SIDE_LENGTH, current_state)
                
                # # Update state after drive
                # current_state = robot.get_state()
                # if current_state:
                #      print(f"Final Drive State: X:{current_state['x']:.3f} Y:{current_state['y']:.3f} Theta:{math.degrees(current_state['theta']):.1f}°")
                
                # # If this is the last side, don't turn
                # if i == len(TARGET_ANGLES) - 1:
                #     break 

                # --- STEP 2: TURN ---
                go_to_angle(robot, target_theta, current_state)
                
                # Update state after turn
                current_state = robot.get_state()
                if current_state:
                    print(f"Final Turn State:  X:{current_state['x']:.3f} Y:{current_state['y']:.3f} Theta:{math.degrees(current_state['theta']):.1f}°")
                
                time.sleep(1)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")

    finally:
        robot.stop()
        robot.close()
        print("\nSquare routine finished/closed.")

if __name__ == "__main__":
    main()
