import sys
import os
import time
import numpy as np
import matplotlib.pyplot as plt

# --- Import Robot Client ---
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../')))
try:
    from Driver.lib.robot_client import RobotClient
except ImportError:
    try:
        from lib.robot_client import RobotClient
    except ImportError:
        from robot_client import RobotClient

# --- MAPPING CONFIGURATION ---
MAP_SIZE_METERS = 4.0      # 4x4 meter area
RESOLUTION = 0.04          # 4cm per grid cell

# --- SENSOR MOUNTING CONFIGURATION (Meters and Radians) ---
# NOTE: These are assumptions for the Ultrasonic (US) sensors.
# They define the position and the angle they point relative to the robot's forward axis (X).

# IR Sensor (Center)
IR_OFFSET_X = 0.10         # 10 cm forward
IR_OFFSET_Y = 0.01         # 1 cm to the left
IR_ANGLE_RAD = 0.0         # 0 degrees (pointing straight forward)

# Ultrasonic Left Sensor
US_L_OFFSET_X = 0.06       # 8 cm forward
US_L_OFFSET_Y = 0.075       # 12 cm to the left
US_L_ANGLE_RAD = np.radians(90) # 45 degrees left (0.785 rad)

# Ultrasonic Right Sensor
US_R_OFFSET_X = 0.06       # 8 cm forward
US_R_OFFSET_Y = -0.075      # 12 cm to the right (negative Y is right)
US_R_ANGLE_RAD = np.radians(-90) # -45 degrees right (-0.785 rad)

# Units are in CM
MAX_IR_DIST_CM = 63        # Max useful distance for IR
MIN_IR_DIST_CM = 3

MAX_US_DIST_CM = 100       # Max useful distance for US (can be higher)
MIN_US_DIST_CM = 5

# --- NUMPY PRINT SETTINGS ---
np.set_printoptions(threshold=sys.maxsize, linewidth=400, formatter={'float': '{: 0.0f}'.format})

def world_to_grid(wx, wy, map_center_idx):
    """Converts real-world meters to grid array indices."""
    gx = int((wx / RESOLUTION) + map_center_idx)
    gy = int((wy / RESOLUTION) + map_center_idx)
    return gx, gy

def save_map(grid_map):
    """Saves the map to PNG and Text files with a timestamp."""
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    
    # 1. Save Image (Visual)
    img_filename = f"grid_map_{timestamp}.png"
    print(f"Saving visual map to: {img_filename}")
    plt.savefig(img_filename)

    # 2. Save Raw Data (Text)
    txt_filename = f"grid_map_{timestamp}.txt"
    print(f"Saving raw data to:   {txt_filename}")
    np.savetxt(txt_filename, np.flipud(grid_map), fmt='%d')

def map_sensor_reading(grid_map, rx, ry, theta, offset_x, offset_y, angle_offset, dist_cm, min_cm, max_cm, center_idx, grid_cells):
    """
    Calculates the global position of an obstacle detected by a single sensor 
    and updates the grid map.
    """
    if min_cm < dist_cm < max_cm:
        
        # 1. Calculate Sensor Global Position (Rotation + Translation)
        # Apply robot heading (theta) to the local sensor offset
        sensor_gx = rx + (offset_x * np.cos(theta) - offset_y * np.sin(theta))
        sensor_gy = ry + (offset_x * np.sin(theta) + offset_y * np.cos(theta))
        
        # 2. Calculate Sensor's True Global Heading
        # Add the sensor's mounting angle to the robot's heading
        global_angle = theta + angle_offset
        
        # 3. Project Obstacle from Sensor Global Position
        dist_m = dist_cm / 100.0
        obs_x = sensor_gx + dist_m * np.cos(global_angle)
        obs_y = sensor_gy + dist_m * np.sin(global_angle)

        # 4. Update Grid
        gx, gy = world_to_grid(obs_x, obs_y, center_idx)
        
        if 0 <= gx < grid_cells and 0 <= gy < grid_cells:
            grid_map[gy, gx] += 1 
            if grid_map[gy, gx] > 10: grid_map[gy, gx] = 10


def main():
    print("Connecting to Robot Service...")
    robot = RobotClient(host="127.0.0.1")

    # 1. Initialize Grid
    grid_cells = int(MAP_SIZE_METERS / RESOLUTION)
    grid_map = np.zeros((grid_cells, grid_cells))
    center_idx = grid_cells // 2

    print(f"Map Initialized: {grid_cells}x{grid_cells} cells")
    print("Mapping... Press Ctrl+C to save and stop.")

    # 2. Setup Plot
    plt.ion() # Interactive mode on
    fig, ax = plt.subplots()
    
    # vmin=0, vmax=10 ensures the grayscale is fixed (0=White, 10=Black)
    img = ax.imshow(grid_map, cmap='Greys', origin='lower', vmin=0, vmax=10,
                    extent=[-MAP_SIZE_METERS/2, MAP_SIZE_METERS/2, -MAP_SIZE_METERS/2, MAP_SIZE_METERS/2])
    
    # Draw Robot Position (Red Dot)
    robot_plot, = ax.plot(0, 0, 'ro', markersize=5, label="Robot") 
    
    # Draw Heading Arrow
    arrow_len = 0.1
    arrow = ax.arrow(0, 0, arrow_len, 0, head_width=0.05, head_length=0.1, fc='blue', ec='blue', zorder=5, label="Heading")

    ax.set_title("Real-Time IR/US Grid Map Fusion")
    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Y (meters)")
    ax.legend()
    
    # Force initial draw
    plt.show(block=False)
    plt.pause(0.1) 

    try:
        while True:
            # 3. Get Data
            state = robot.get_state()
            if state is None:
                time.sleep(0.05)
                continue

            # 4. Extract & Convert
            rx = state['x']
            ry = state['y']
            theta = state['theta']
            
            # Sensor Readings
            ir_cm = state['ir']
            us_l_cm = state['us_l']
            us_r_cm = state['us_r']

            # --- MAP SENSOR READINGS ---
            
            # Map IR (Center)
            map_sensor_reading(grid_map, rx, ry, theta, 
                               IR_OFFSET_X, IR_OFFSET_Y, IR_ANGLE_RAD, 
                               ir_cm, MIN_IR_DIST_CM, MAX_IR_DIST_CM, 
                               center_idx, grid_cells)

            # Map Ultrasonic Left
            map_sensor_reading(grid_map, rx, ry, theta, 
                               US_L_OFFSET_X, US_L_OFFSET_Y, US_L_ANGLE_RAD, 
                               us_l_cm, MIN_US_DIST_CM, MAX_US_DIST_CM, 
                               center_idx, grid_cells)

            # Map Ultrasonic Right
            map_sensor_reading(grid_map, rx, ry, theta, 
                               US_R_OFFSET_X, US_R_OFFSET_Y, US_R_ANGLE_RAD, 
                               us_r_cm, MIN_US_DIST_CM, MAX_US_DIST_CM, 
                               center_idx, grid_cells)


            # 5. Update Visualization
            robot_plot.set_data([rx], [ry]) 
            img.set_data(grid_map)
            
            # Update Arrow
            arrow.remove()
            dx = arrow_len * np.cos(theta)
            dy = arrow_len * np.sin(theta)
            # Re-draw arrow
            arrow = ax.arrow(rx, ry, dx, dy, head_width=0.05, head_length=0.1, fc='blue', ec='blue', zorder=5)

            plt.pause(0.01) 

    except KeyboardInterrupt:
        print("\nMapping Stopped.")
        #save_map(grid_map)
        print("-" * 40)

    finally:
        robot.close()
        plt.close('all')

if __name__ == "__main__":
    main()
