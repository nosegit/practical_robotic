import sys
import os
import time
import math
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))

from Driver.lib.robot_client import RobotClient

linear_speed = 0.05

def main():
    robot = RobotClient(host="localhost")
    print("Connected to robot.")

    robot._wait_for_valid_state()
    print("Connected to robot and received valid state.")
    robot.reset_pose()
    print("Pose reset to (0,0,0).")
    time.sleep(1.0)

    for n in range(5):
            # Move forward
            robot.move_distance(distance_meters=0.1, max_speed=linear_speed)

            # Stop before turning
            robot.set_velocity(0.0, 0.0)
            time.sleep(0.5)

            # Turn 180 degrees
            robot.rotate_to_angle(180)
            
            # Stop before turning
            robot.set_velocity(0.0, 0.0)
            time.sleep(0.5)

            # Move forward
            robot.move_distance(distance_meters=0.1, max_speed=linear_speed)

            # Turn 180 degrees
            robot.rotate_to_angle(0)
            time.sleep(0.5)

    print("Square routine finished.")
    robot.rotate_to_angle(0, speed= 0.5)
    
    state = robot._wait_for_valid_state()
    print(f'x:{state["x"]}, y:{state["y"]}, theta: {state["theta"]}')

if __name__ == "__main__":
    main()