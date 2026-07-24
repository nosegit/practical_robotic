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
        state = None
        while state is None:
            state = robot._wait_for_valid_state()
            time.sleep(0.1)

        robot.reset_pose()
        
        # print(f"Connected! Initial Pos: X={state['x']:.2f}, Y={state['y']:.2f}")
        # # robot.set_theta(angle_degrees=90)
        # # time.sleep(1000)

        for i in range(4):
            print(f"============= round : {i} =============")
            for j in range(4) :
                # robot.rotate(angle_degrees=90)
                robot.rotate_to_angle(target_angle_degrees=j*90)
                # if(robot._wait_for_valid_state()['ir'] < 20) : 
                #     robot.wall_sweep(sweep_arc=20, speed = 0.1)
                #     robot.set_theta(angle_degrees=robot.get_snap_angle())

        #         time.sleep(1)
        # robot.rotate_to_angle(target_angle_degrees=0)

        # robot.rotate_to_angle(target_angle_degrees=90)
            

        # robot.rotate(angle_degrees=90)
        # if(robot._wait_for_valid_state()['ir'] < 20) : 
        #     robot.wall_sweep(sweep_arc=5, speed = 0.1)
        #     robot.set_theta(angle_degrees=robot.get_snap_angle())



    except KeyboardInterrupt:
        print("\nInterrupted by user!")
        robot.stop()

    finally:
        print("Closing connection.")
        robot.close()

if __name__ == "__main__":
    main()