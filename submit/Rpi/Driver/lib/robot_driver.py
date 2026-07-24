"""
RobotDriver: Low-level serial driver for robot hardware communication.
Handles bidirectional communication via USB serial: sends velocity commands and receives telemetry.
Uses threading for non-blocking read operations with thread-safe state access.
"""
import serial
import struct
import threading
import time
from dataclasses import dataclass

SERIAL_PORT = '/dev/ttyUSB0' # arduino serial port
BAUD_RATE = 115200

'''
packet structure:
Telemetry Packet (from robot to host): 22 bytes total
- Start Byte: 0xA5 (1 byte)
- Timestamp: uint32 (4 bytes)
- X Position: int16 (2 bytes, in mm)
- Y Position: int16 (2 bytes, in mm)
- Orientation (Theta): int16 (2 bytes, in 0.0001 rad)
- Linear Velocity (v): int16 (2 bytes, in mm/s)
- Angular Velocity (w): int16 (2 bytes, in mm/s)
- Ultrasonic Sensor Left (us_l): uint16 (2 bytes, in mm)
- Ultrasonic Sensor Right (us_r): uint16 (2 bytes, in mm)
- Infrared Sensor (ir): uint16 (2 bytes, raw value)
- Checksum: uint8 (1 byte, XOR of all payload bytes)

Command Packet (from host to robot): 6 bytes total
- Start Byte: 0x5A (1 byte)
- Linear Velocity (v): int16 (2 bytes, in mm/s)
- Angular Velocity (w): int16 (2 bytes, in mm/s)
- Checksum: uint8 (1 byte, XOR of all payload bytes)

'''
# Protocol Constants
TELEMETRY_FMT = '<I5h3H' 
PACKET_SIZE = 22
COMMAND_FMT = '<2h'

@dataclass
class RobotState:
    """Data class holding current robot state: pose, velocity, and sensor readings."""
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
        """
        Initialize serial connection and prepare background read thread.
        Does not start reading until start() is called.
        
        Args:
            port: Serial port device path (e.g., '/dev/ttyUSB0' on Linux)
            baud: Baud rate for serial communication (default: 115200)
        """
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.lock = threading.Lock()
        self.running = False
        self.latest_state = RobotState()
        self.thread = threading.Thread(target=self._read_loop)
        self.thread.daemon = True

    def start(self):
        """Begin reading telemetry in background thread."""
        self.running = True
        self.thread.start()

    def close(self):
        """
        Safe shutdown: signal thread to stop, wait for completion, then close port.
        Critical to call this before program exit to prevent resource leaks.
        """
        self.running = False # Signal thread to stop
        
        # 1. Wait for thread to finish (CRITICAL FIX)
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
            
        # 2. Now it's safe to close the port
        if self.ser.is_open:
            self.ser.close()

    def send_velocity(self, v, w):
        """
        Send velocity command to robot with XOR checksum protection.
        
        Args:
            v: Linear velocity (m/s, scaled to int internally)
            w: Angular velocity (rad/s, scaled to int internally)

        data format:
        <2h
        0: v (int16, mm/s)
        1: w (int16, mm/s)
        
        """
        try:
            payload = struct.pack(COMMAND_FMT, int(v*1000), int(w*1000))
            cs = 0
            for b in payload: cs ^= b
            
            with self.lock:
                if self.ser.is_open:
                    self.ser.write(b'\x5A' + payload + bytes([cs]))
        except Exception as e:
            print(f"[Driver] Send Error: {e}")

    def get_state(self):
        """Thread-safe getter for latest robot state. Returns immediately with current snapshot."""
        with self.lock: return self.latest_state

    def _read_loop(self):
        """
        Background thread main loop: continuously read packets from serial port.
        Validates packet sync byte and checksum before parsing state updates.
        """
        while self.running:
            try:
                # Check if port is open before accessing property
                if not self.ser.is_open:
                    break
                    
                if self.ser.in_waiting >= PACKET_SIZE:
                    if self.ser.read(1) != b'\xA5': continue
                    data = self.ser.read(PACKET_SIZE - 1)
                    if len(data) != PACKET_SIZE - 1: continue
                    
                    payload = data[:-1]
                    if self._calc_checksum(payload) == data[-1]:
                        self._parse(payload)
                else:
                    time.sleep(0.002)
            except Exception:
                # Silently ignore errors during shutdown
                if not self.running: break
                pass

    def _calc_checksum(self, payload):
        """Calculate XOR checksum of payload bytes."""
        cs = 0
        for b in payload: cs ^= b
        return cs

    def _parse(self, payload):
        """
        Unpack binary telemetry payload and update robot state with scaling factors.
        All coordinates and velocities are received as integers and scaled appropriately.

        data format:
        <I5h3H
        0: timestamp (uint32)
        1: x (int16, mm)
        2: y (int16, mm)
        3: theta (int16, 0.0001 rad)
        4: v (int16, mm/s)
        5: w (int16, mm/s)
        6: us_l (uint16, mm)
        7: us_r (uint16, mm)
        8: ir (uint16, raw)

        """
        try:
            u = struct.unpack(TELEMETRY_FMT, payload)
            with self.lock:
                self.latest_state.timestamp = u[0]
                self.latest_state.x = u[1] / 1000.0
                self.latest_state.y = u[2] / 1000.0
                self.latest_state.theta = u[3] / 10000.0
                self.latest_state.v = u[4] / 1000.0
                self.latest_state.w = u[5] / 1000.0
                self.latest_state.us_l = u[6]
                self.latest_state.us_r = u[7]
                self.latest_state.ir = u[8]
        except: pass