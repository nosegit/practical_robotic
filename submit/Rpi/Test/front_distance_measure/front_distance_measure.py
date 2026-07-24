'''
function for measuring front obstacle distance
    - Front IR distance
    - Aruco tag distance


'''


import sys
import os
import time

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))
from Driver.lib.robot_client import RobotClient
from Vision.lib.camera_client import CameraClient

def main():
    robot = RobotClient()
    camera = CameraClient()

    while True:
        state = robot._wait_for_valid_state()
        tags = camera.get_all_marker()

        ir_dist = 999
        tag_dist = 999

        if(state) :
            ir_dist = state['ir']

        if(tags):
            tag_dist = min([n['distance'] for n in tags])

        print(f'IR: {ir_dist:.2f}, TAG: {tag_dist:.2f}')

        time.sleep(0.1)

if __name__ == "__main__":
    main()