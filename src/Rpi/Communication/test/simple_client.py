import sys
import os
# Add parent directory to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import time
from lib.robot_client import RobotClient

def main():
    # 1. Connect
    print("Connecting to Robot Service...")
    robot = RobotClient(host="localhost")
                

    try:
        state = robot.get_state()
        while state is None:
            state = robot.get_state()

        print(f"X: {state['x']:.3f}, Y: {state['y']:.3f}, Theta: {state['theta']:.3f}, ir: {state['ir']}")

        print("Starting Square Path Routine...")
        
        # Move Forward
        robot.set_velocity(0.05, 0.0) 
        time.sleep(1)
        state = robot.get_state()
        print(f"X: {state['x']:.3f}, Y: {state['y']:.3f}, Theta: {state['theta']:.3f}, ir: {state['ir']}")

        for i in range(3):       
            # Turn
            robot.set_velocity(0.0, 0.157)
            time.sleep(1)
            robot.set_velocity(0.0, 0.0)
            state = robot.get_state()
            print(f"X: {state['x']:.3f}, Y: {state['y']:.3f}, Theta: {state['theta']:.3f}, ir: {state['ir']}")
            time.sleep(0.5)

        for i in range(3):  
            # Turn
            robot.set_velocity(0.0, -0.157)
            time.sleep(1)
            robot.set_velocity(0.0, 0.0)
            state = robot.get_state()
            print(f"X: {state['x']:.3f}, Y: {state['y']:.3f}, Theta: {state['theta']:.3f}, ir: {state['ir']}")
            time.sleep(0.5)

        robot.set_velocity(-0.05, 0.0) 
        time.sleep(1)
        state = robot.get_state()
        print(f"X: {state['x']:.3f}, Y: {state['y']:.3f}, Theta: {state['theta']:.3f}, ir: {state['ir']}")

        
        # Stop
        robot.set_velocity(0.0, 0.0)

    except KeyboardInterrupt:
        print("Interrupted!")

    finally:
        # 5. Clean Exit
        print("Stopping Robot...")
        robot.stop()
        robot.close()
        exit(0)

if __name__ == "__main__":
    main()