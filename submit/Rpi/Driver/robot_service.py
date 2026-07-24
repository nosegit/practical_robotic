import time
import zmq
import json
import sys
import os
import logging
from dataclasses import asdict

# Adjust import path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from lib.robot_driver import RobotDriver


# --- CONFIGURATION ---
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
HOST_IP = "0.0.0.0"
PUB_PORT = 5555 # port that publish robot data (public/subscribe)
REP_PORT = 5556 # port that request robot command (requeat/reply)

# --- LOGGING SETUP ---
# Options: logging.DEBUG (Show everything), logging.INFO (Show important events), logging.WARNING (Show only errors)
LOG_LEVEL = logging.DEBUG 

logging.basicConfig(
    level=LOG_LEVEL,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger("RobotService")

def main():
    logger.info(f"Initializing Robot Service on {SERIAL_PORT}...")
    
    # 1. Start Hardware Driver
    driver = RobotDriver(port=SERIAL_PORT, baud=BAUD_RATE)
    try:
        driver.start()
        logger.info("Hardware Driver Started Successfully.")
    except Exception as e:
        logger.critical(f"Failed to start driver: {e}")
        return

    # 2. Setup ZeroMQ Context
    context = zmq.Context()
    
    # Socket A: TELEMETRY (PUB)
    try:
        pub = context.socket(zmq.PUB)
        pub.bind(f"tcp://{HOST_IP}:{PUB_PORT}")
        logger.info(f"Telemetry Publisher bound to {HOST_IP}:{PUB_PORT}")
    except zmq.ZMQError as e:
        logger.critical(f"Could not bind PUB port {PUB_PORT}: {e}")
        return
    
    # Socket B: COMMANDS (REP)
    try:
        rep = context.socket(zmq.REP)
        rep.bind(f"tcp://{HOST_IP}:{REP_PORT}")
        logger.info(f"Command Listener bound to {HOST_IP}:{REP_PORT}")
    except zmq.ZMQError as e:
        logger.critical(f"Could not bind REP port {REP_PORT}: {e}")
        return

    logger.info("Service is running. Press Ctrl+C to stop.")

    last_cmd_time = time.time()
    tick_count = 0

    try:
        while True:
            # --- A. Publish State (Continuous) ---
            state = driver.get_state()
            pub.send_string(json.dumps(asdict(state)))

            # Heartbeat: Logged as DEBUG. Won't show in INFO mode.
            # We keep the % 100 check so even in Debug mode it prints every 2s, not 50Hz.
            if tick_count % 100 == 0:
                logger.debug(f"Heartbeat | Time: {state.timestamp} | X: {state.x:.2f} | Y: {state.y:.2f} | Theta: {state.theta:.2f} | us_l : {state.us_l:.2f} | us_r: {state.us_r:.2f}| IR: {state.ir:.2f}")
            tick_count += 1

            # --- B. Handle Commands (Non-Blocking) ---
            try:
                msg = rep.recv_json(flags=zmq.NOBLOCK)
                
                v = msg.get('v', 0.0)
                w = msg.get('w', 0.0)
                
                # Log commands as DEBUG to prevent spamming console during high-frequency control
                logger.debug(f"CMD Received: v={v:.3f}, w={w:.3f}")
                
                driver.send_velocity(v, w)
                last_cmd_time = time.time()
                
                rep.send_string("ACK") 
                
            except zmq.Again:
                pass

            time.sleep(0.02) # 50hz

    except KeyboardInterrupt:
        logger.info("Stopping Service...")
    
    except Exception as e:
        logger.exception("Critical Runtime Error")

    finally:
        logger.info("Closing Resources...")
        driver.close()
        pub.close()
        rep.close()
        context.term()
        logger.info("Done.")

if __name__ == "__main__":
    main()
