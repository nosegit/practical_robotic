import time
import zmq
import json
from dataclasses import asdict
from lib.robot_driver import RobotDriver


SERIAL_PORT = 'COM7'
BAUD_RATE = 115200

def main():
    driver = RobotDriver(port=SERIAL_PORT, baud=BAUD_RATE)
    try:
        driver.start()
        print("Driver Started.")
    except Exception as e:
        print(f"Failed to start driver: {e}")
        return

    context = zmq.Context()
    pub = context.socket(zmq.PUB)
    pub.bind("tcp://0.0.0.0:5555")
    
    rep = context.socket(zmq.REP)
    rep.bind("tcp://0.0.0.0:5556")

    print("ZMQ Service Running...")

    try:
        while True:
            # 1. Publish State
            state = driver.get_state()
            pub.send_string(json.dumps(asdict(state)))

            # print("Published State:", asdict(state))
            # time.sleep(0.02) # 50Hz

            # 2. Handle Commands
            try:
                msg = rep.recv_json(flags=zmq.NOBLOCK)

                # --- DEBUG PRINT START ---
                # v = msg.get('v', 0.0)
                # w = msg.get('w', 0.0)
                # print(f"DEBUG: ZMQ Received v={v}, w={w}") 
                # --- DEBUG PRINT END ---

                driver.send_velocity(msg.get('v', 0), msg.get('w', 0))
                rep.send_string("ACK")
            except zmq.Again:
                pass

            time.sleep(0.02) # 50Hz
    except KeyboardInterrupt:
        driver.close()

if __name__ == "__main__":
    main()