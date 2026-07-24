import serial
import struct
import threading
import time
from dataclasses import dataclass

SERIAL_PORT = '/dev/ttyUSB0' # Check your port
BAUD_RATE = 115200

# Protocol Constants
TELEMETRY_FMT = '<I5h3H' 
PACKET_SIZE = 22
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
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.lock = threading.Lock()
        self.running = False
        self.latest_state = RobotState()
        self.thread = threading.Thread(target=self._read_loop)
        self.thread.daemon = True

    def start(self):
        self.running = True
        self.thread.start()

    def close(self):
        """Clean shutdown to prevent errors."""
        self.running = False # Signal thread to stop
        
        # 1. Wait for thread to finish (CRITICAL FIX)
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
            
        # 2. Now it's safe to close the port
        if self.ser.is_open:
            self.ser.close()

    def send_velocity(self, v, w):
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
        with self.lock: return self.latest_state

    def _read_loop(self):
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
        cs = 0
        for b in payload: cs ^= b
        return cs

    def _parse(self, payload):
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