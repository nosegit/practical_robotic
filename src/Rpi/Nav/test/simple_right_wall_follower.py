import sys
import os
import time

# Add parent directory to path to import the library
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../')))
from Driver.lib.robot_client import RobotClient

# --- CONFIGURATION ---
TARGET_WALL_DIST = 5.5  # Desired distance from right wall (cm)
FRONT_STOP_DIST = 4.0   # Minimum front distance before turning (cm)
WALL_LOST_DIST = 55.0    # If US_R is above this, we assume we lost the wall (cm)

# SENSOR RANGES (User Provided)
# IR: [3, 63)
# US: [5, 63)

# PID GAINS
KP = 0.1  # Proportional Gain (How sharply to turn based on error)
CRUISE_SPEED = 0.05

def main():
    robot = RobotClient(host="127.0.0.1")
    robot._wait_for_valid_state()
    print("Connected. Starting Right Wall Follower...")

    robot.reset_pose()

    try:
        while True:
            # 1. Get Sensor Data
            state = robot.get_state()
            if state is None:
                time.sleep(0.01)
                continue
            


            # Safely get sensor values, defaulting to MAX range if missing
            # (Max range 63 implies "open space")
            us_r = state.get('us_r', 63.0)
            ir = state.get('ir', 63.0)

            # Debug output (optional, slows down console)
            # print(f"IR: {ir:.1f} | US_R: {us_r:.1f}")

            # --- BEHAVIOR TREE ---

            # CASE 1: OBSTACLE AHEAD (Inner Corner)
            # If front is blocked, we MUST turn Left to avoid collision.
            if ir < FRONT_STOP_DIST:
                print(f"Front Blocked ({ir}cm). Turning Left.")
                # Zero linear velocity, rotate Left (+W)
                robot.rotate(90)
                robot.rotate_to_angle(robot.get_snap_angle())

            
            # CASE 2: WALL LOST (Outer Corner / Open Space)
            # If US_R is near max range (63), we lost the wall.
            elif us_r >= WALL_LOST_DIST:
                print("Wall Lost. Arcing Right to find it.")
                # Move forward + Turn Right (-W) to wrap around corner
                # robot.set_velocity(0.075, -0.5)

                # forward + right 90 degree + forware
                robot.rotate_to_angle(robot.get_snap_angle())
                robot.move_distance(distance_meters=0.15)
                robot.rotate(-90)
                robot.rotate_to_angle(robot.get_snap_angle())

                break



            # CASE 3: WALL FOLLOWING (PID Control)
            else:
                # Calculate Error: Positive Error = Too Close, Negative Error = Too Far
                # Example: Target 15, Actual 10 (Too Close). Error = 15 - 10 = 5.
                error = TARGET_WALL_DIST - us_r
                
                # Control Law:
                # If Error is +5 (Too Close), we want to turn LEFT (+W).
                # If Error is -5 (Too Far), we want to turn RIGHT (-W).
                # Therefore: W = Kp * Error
                
                angular_correction = KP * error
                
                # Clamp the rotation speed to avoid spinning out
                angular_correction = max(min(angular_correction, 0.6), -0.6)
                
                # print(f"Following. Err: {error:.1f} | W: {angular_correction:.2f}")
                robot.set_velocity(CRUISE_SPEED, angular_correction)
                # robot.rotate_to_angle(robot.get_snap_angle())
                # ir = robot._wait_for_valid_state()['ir']
                # robot.move_distance(distance_meters = min(0.30, (ir-FRONT_STOP_DIST)/100 + 0.005 ),
                #                     speed = CRUISE_SPEED)
                

            # Small sleep to keep loop stable
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("Stopping robot...")
        robot.stop()
    finally:
        robot.close()

if __name__ == "__main__":
    main()