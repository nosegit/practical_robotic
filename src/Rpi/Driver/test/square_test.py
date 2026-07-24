import sys
import os
import time
import math
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../')))

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

    for side in range(4):
        print(f"Starting side {side + 1} of the square.")

        # Move forward
        robot.move_distance(distance_meters=0.3, max_speed=linear_speed)

        # Stop before turning
        robot.set_velocity(0.0, 0.0)
        time.sleep(0.5)

        # Turn 90 degrees
        # robot.set_velocity(0.0, math.pi / 4)  # Turn at 45 degrees/sec
        # time.sleep(2.0)  # Turn for 2 seconds to achieve ~90 degrees
        robot.rotate_to_angle((side + 1) * (90), speed= 0.5)

        # Stop after turning
        robot.set_velocity(0.0, 0.0)
        time.sleep(0.5)

    print("Square routine finished.")

if __name__ == "__main__":
    main()