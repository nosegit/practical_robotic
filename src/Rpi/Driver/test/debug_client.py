import sys
import os
# Add parent directory to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import time
from lib.robot_client import RobotClient

def main():
    robot = RobotClient(host="localhost")

    while True:
        state = robot.get_state()

        if state:
            print(f"Initial State: X: {state['x']:.3f}, Y: {state['y']:.3f}, Theta: {state['theta']:.3f}, ir: {state['ir']} , us_l: {state['us_l']} , us_r: {state['us_r']}")
        else:
            print("No initial state received.")

        time.sleep(0.1)

if __name__ == "__main__":
    main()