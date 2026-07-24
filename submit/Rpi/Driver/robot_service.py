"""
RobotService: Central ZMQ-based service bridging low-level hardware driver and high-level clients.
Architecture:
  - Hardware Driver (lib.robot_driver): Manages USB serial communication with robot firmware
  - ZMQ PUB Socket (port 5555): Broadcasts robot telemetry (pose, velocity, sensors) at 50 Hz
  - ZMQ REP Socket (port 5556): Receives velocity commands from clients and replies with ACK
  - Threading: Hardware reads run in background; main thread handles ZMQ pub/sub loop

Usage:
  1. Run this service on Raspberry Pi with robot connected
  2. Clients connect to localhost:5555 (subscribe telemetry) and localhost:5556 (send commands)
  3. Press Ctrl+C to gracefully shutdown all connections
"""
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
    """
    Main service loop: initialize hardware and ZMQ sockets, then continuously:
    1. Publish robot state to all subscribers
    2. Listen for and execute velocity commands from clients
    3. Gracefully handle shutdown on Ctrl+C
    """
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
    # Broadcasts state to all connected clients at 50Hz
    try:
        pub = context.socket(zmq.PUB)
        pub.bind(f"tcp://{HOST_IP}:{PUB_PORT}")
        logger.info(f"Telemetry Publisher bound to {HOST_IP}:{PUB_PORT}")
    except zmq.ZMQError as e:
        logger.critical(f"Could not bind PUB port {PUB_PORT}: {e}")
        return
    
    # Socket B: COMMANDS (REP)
    # Receives velocity commands and replies with ACK
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
            # Fetch latest state from hardware driver (thread-safe) and broadcast as JSON
            state = driver.get_state()
            pub.send_string(json.dumps(asdict(state)))

            # Heartbeat logging (DEBUG level): prints every 100 ticks (~2 seconds at 50Hz)
            # Helps verify service is running without spamming console
            if tick_count % 100 == 0:
                logger.debug(f"Heartbeat | Time: {state.timestamp} | X: {state.x:.2f} | Y: {state.y:.2f} | Theta: {state.theta:.2f} | us_l : {state.us_l:.2f} | us_r: {state.us_r:.2f}| IR: {state.ir:.2f}")
            tick_count += 1

            # --- B. Handle Commands (Non-Blocking) ---
            # zmq.NOBLOCK prevents main loop from stalling on command socket
            # If no command available, zmq.Again exception is caught silently
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

            time.sleep(0.02) # Main loop runs at 50Hz (20ms per iteration)

    except KeyboardInterrupt:
        logger.info("Stopping Service...")
    
    except Exception as e:
        logger.exception("Critical Runtime Error")

    finally:
        # Cleanup: close all sockets and hardware driver to free resources
        logger.info("Closing Resources...")
        driver.close()
        pub.close()
        rep.close()
        context.term()
        logger.info("Done.")

if __name__ == "__main__":
    main()
