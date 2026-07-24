import sys
import os
import time

# Add parent directory to path so we can import the library
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
# Import the client class
from lib.robot_client import RobotClient


def main():
    print("Connecting to Robot Service...")
    # Change host to "192.168.X.X" if connecting from a different machine
    robot = RobotClient(host="127.0.0.1")

    try:
        # 1. Wait for Valid Data
        print("Waiting for telemetry stream...")
        state = robot._wait_for_valid_state()
        print(f"Connected! Initial Pos: X={state['x']:.2f}, Y={state['y']:.2f}")
        print("Starting Pattern Routine...")

        # --- USER PATTERN START ---

        # 1. Move Forward (1s)
        print("Moving Forward...")
        robot.set_velocity(0.05, 0.0) 
        time.sleep(1)
        
        state = robot.get_state()
        if state:
            print(f"X: {state['x']:.3f}, Y: {state['y']:.3f}, Theta: {state['theta']:.3f}, ir: {state['ir']}")

        # 2. Loop: Turn Left 3 times
        print("Starting Left Turn Loop...")
        for i in range(3):       
            robot.set_velocity(0.0, 0.157) # Turn
            time.sleep(1)
            
            robot.set_velocity(0.0, 0.0)   # Stop briefly
            state = robot.get_state()
            if state:
                print(f"[{i+1}/3] X: {state['x']:.3f}, Y: {state['y']:.3f}, Theta: {state['theta']:.3f}, ir: {state['ir']}")
            time.sleep(0.5)

        # 3. Loop: Turn Right 3 times
        print("Starting Right Turn Loop...")
        for i in range(3):  
            robot.set_velocity(0.0, -0.157) # Turn opposite way
            time.sleep(1)
            
            robot.set_velocity(0.0, 0.0)    # Stop briefly
            state = robot.get_state()
            if state:
                print(f"[{i+1}/3] X: {state['x']:.3f}, Y: {state['y']:.3f}, Theta: {state['theta']:.3f}, ir: {state['ir']}")
            time.sleep(0.5)

        # 4. Move Backward
        print("Moving Backward...")
        robot.set_velocity(-0.05, 0.0) 
        time.sleep(1)
        
        state = robot.get_state()
        if state:
            print(f"X: {state['x']:.3f}, Y: {state['y']:.3f}, Theta: {state['theta']:.3f}, ir: {state['ir']}")

        # --- USER PATTERN END ---

        # Final Stop
        print("Pattern Complete. Stopping.")
        robot.set_velocity(0.0, 0.0)

    except KeyboardInterrupt:
        print("\nInterrupted by user!")
        robot.stop()

    finally:
        print("Closing connection.")
        robot.close()

if __name__ == "__main__":
    main()