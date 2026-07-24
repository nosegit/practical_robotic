import serial
import struct
import threading
import time
from dataclasses import dataclass

# --- CONFIGURATION ---
SERIAL_PORT = 'COM7' # Adjust for Pi (often ttyACM0 for Arduino via USB)
BAUD_RATE = 115200

# --- PROTOCOL CONSTANTS ---
# Formats: < = Little Endian
# Telemetry: I (uint32), h (int16), H (uint16)
# Structure: Start(1) + Time(4) + x(2) + y(2) + th(2) + v(2) + w(2) + usl(2) + usr(2) + ir(2) + CS(1)
TELEMETRY_FMT = '<I5h3H' 
TELEMETRY_SIZE = 20 # Payload size excluding Start and Checksum
PACKET_SIZE = 22    # Total size

# Command: Start(1) + v(2) + w(2) + CS(1)
COMMAND_FMT = '<2h'

@dataclass
class RobotState:
    timestamp: int = 0
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    v: float = 0.0
    w: float = 0.0
    us_l: int = 0
    us_r: int = 0
    ir: int = 0

class RobotDriver:
    def __init__(self, port=SERIAL_PORT, baud=BAUD_RATE):
        self.ser = serial.Serial(port, baud, timeout=1)
        self.lock = threading.Lock()
        self.running = False
        self.latest_state = RobotState()
        self.packet_count = 0
        
        # Start reading thread
        self.thread = threading.Thread(target=self._read_loop)
        self.thread.daemon = True

    def start(self):
        self.running = True
        self.thread.start()
        print(f"Connected to Robot on {self.ser.port}")

    def send_velocity(self, v_m_s, w_rad_s):
        """
        Send velocity command.
        v_m_s: Linear velocity in Meters/sec
        w_rad_s: Angular velocity in Rad/sec
        """
        # 1. Scale to integers
        v_mm = int(v_m_s * 1000)
        w_mrad = int(w_rad_s * 1000)

        # 2. Pack Payload
        try:
            payload = struct.pack(COMMAND_FMT, v_mm, w_mrad)
        except struct.error as e:
            print(f"Packing error: {e}")
            return

        # 3. Calculate Checksum (XOR)
        checksum = 0
        for byte in payload:
            checksum ^= byte

        # 4. Construct Packet: Start(0x5A) + Payload + Checksum
        packet = b'\x5A' + payload + bytes([checksum])
        
        with self.lock:
            self.ser.write(packet)

    def get_state(self):
        """Returns the latest telemetry snapshot thread-safely."""
        with self.lock:
            return self.latest_state

    def _read_loop(self):
        """
        Continuous loop to parse incoming binary telemetry.
        Implements a seek-and-read mechanism for sync bytes.
        """
        while self.running:
            if self.ser.in_waiting >= PACKET_SIZE:
                # 1. Read one byte to check for Sync (0xA5)
                # Note: In high throughput, reading 1 by 1 is slow in Python.
                # Ideally, read a chunk and process, but for 30Hz this is fine.
                try:
                    sync = self.ser.read(1)
                    if sync != b'\xA5':
                        continue # Skip garbage

                    # 2. Read remainder of packet
                    data = self.ser.read(PACKET_SIZE - 1)
                    if len(data) != PACKET_SIZE - 1:
                        continue

                    # 3. Split into payload and checksum
                    payload = data[:-1]
                    received_checksum = data[-1]

                    # 4. Validate Checksum
                    calc_checksum = 0
                    # XOR needs to include timestamp through sensors
                    # Note: Python's 'payload' here corresponds to the struct fields
                    for byte in payload:
                        calc_checksum ^= byte

                    if calc_checksum == received_checksum:
                        self._parse_telemetry(payload)
                    else:
                        print("Checksum mismatch!")
                        print(payload)

                except Exception as e:
                    print(f"Serial Error: {e}")

            else:
                time.sleep(0.005) # Prevent CPU hogging

    def _parse_telemetry(self, payload):
        """Unpacks binary data into the dataclass."""
        try:
            unpacked = struct.unpack(TELEMETRY_FMT, payload)
            
            with self.lock:
                self.latest_state.timestamp = unpacked[0]
                self.latest_state.x = unpacked[1] / 1000.0
                self.latest_state.y = unpacked[2] / 1000.0
                self.latest_state.theta = unpacked[3] / 10000.0
                self.latest_state.v = unpacked[4] / 1000.0
                self.latest_state.w = unpacked[5] / 1000.0
                self.latest_state.us_l = unpacked[6]
                self.latest_state.us_r = unpacked[7]
                self.latest_state.ir = unpacked[8]
                self.packet_count += 1
                
        except struct.error:
            print("Unpacking error")

# --- MAIN EXECUTION EXAMPLE ---
if __name__ == "__main__":
    robot = RobotDriver()
    try:

        robot.start()
        
        print("Starting Square Path Routine...")
        # Move Forward
        robot.send_velocity(0.05, 0.0) 
        time.sleep(1)
        
        # Turn
        robot.send_velocity(0.0, 1.57)
        time.sleep(1)
        
        # Stop
        robot.send_velocity(0.0, 0.0)
        
        while True:
            state = robot.get_state()
            print(f"State: X={state.x:.2f} Y={state.y:.2f} Theta={state.theta:.2f} | Packets: {robot.packet_count}")
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        robot.send_velocity(0, 0)
        print("Stopping.")