import sys
import os
import time

# --- 1. Import Robot Client ---
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
try:
    from lib.robot_client import RobotClient
except ImportError:
    # Fallback if in the same folder
    from robot_client import RobotClient

# --- 2. Cross-Platform Getch (Input Handler) ---
class _Getch:
    """Gets a single character from standard input.  Does not echo to the screen."""
    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()

    def __call__(self): return self.impl()

class _GetchUnix:
    def __init__(self):
        import tty, sys

    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

class _GetchWindows:
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        return msvcrt.getch().decode('utf-8')

get_key = _Getch()

# --- 3. Configuration ---
LINEAR_SPEED = 0.1  # m/s
ANGULAR_SPEED = 0.5 # rad/s

def print_instructions():
    print("="*30)
    print("      ROBOT TELEOP      ")
    print("="*30)
    print("   [W]   ")
    print("[A][S][D]    Move Robot")
    print("[Z]      Reset Odometry")
    print("[X]      360 rotate")
    print("[C]      Set Snap angle")
    print("[V]      Sweep angle")
    print("[SPACE]  Stop")
    print("[Q]      Quit")
    print("="*30)

def main():
    print("Connecting to Robot Service...")
    # Change IP if needed (e.g., "192.168.1.50")
    robot = RobotClient(host="127.0.0.1")
    
    # Wait for connection
    state = None
    while state is None:
        state = robot.get_state()
        time.sleep(0.1)
    
    print_instructions()

    current_v = 0.0
    current_w = 0.0

    try:
        while True:
            # 1. Read Key (Blocking)
            key = get_key().lower()

            # 2. Map Key to Velocity
            if key == 'w':
                current_v = LINEAR_SPEED
                current_w = 0.0
                print(f"\rFORWARD  (v={current_v})", end="")
            
            elif key == 's':
                current_v = -LINEAR_SPEED
                current_w = 0.0
                print(f"\rBACKWARD (v={current_v})", end="")

            elif key == 'a':
                current_v = 0.0
                current_w = ANGULAR_SPEED
                print(f"\rLEFT     (w={current_w})", end="")

            elif key == 'd':
                current_v = 0.0
                current_w = -ANGULAR_SPEED
                print(f"\rRIGHT    (w={current_w})", end="")

            elif key == ' ':
                current_v = 0.0
                current_w = 0.0
                print(f"\rSTOPPED                  ", end="")

            elif key == 'x':
                for i in range(4):
                    # robot.rotate(angle_degrees=90) 
                    robot.rotate_to_angle(target_angle_degrees=i*90)
                    # if(robot._wait_for_valid_state()['ir'] < 20) : 
                    #     robot.wall_sweep(sweep_arc=20, speed = 0.1)   
                    #     robot.set_theta(angle_degrees=robot.get_snap_angle())

                    time.sleep(1)
                robot.rotate_to_angle(target_angle_degrees=0)
            
            elif key == 'v':
                robot.wall_sweep(sweep_arc=30, speed = 0.2)

            elif key == 'c':   
                robot.set_theta(angle_degrees=robot.get_snap_angle())

            elif key == 'z':
                robot.reset_pose()
                print("\nOdometry Reset.")

            elif key == 'q':
                print("\nQuitting...")
                break

            # 3. Send Command
            robot.set_velocity(current_v, current_w)
        
            print("\n"*20)
            print_instructions()
            
            # Optional: Print telemetry for feedback
            # state = robot.get_state()
            # if state:
            #     print(f" | X={state['x']:.2f} Y={state['y']:.2f}", end="")

    except KeyboardInterrupt:
        print("\nInterrupted.")

    finally:
        robot.stop()
        robot.close()

if __name__ == "__main__":
    main()
