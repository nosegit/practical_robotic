import sys
import os
import time
import numpy as np
import matplotlib.pyplot as plt

# --- Import Robot Client ---
# Modified to look for Driver.lib structure based on your setup
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
RESOLUTION = 0.04          # 5cm per grid cell
SENSOR_OFFSET = 0.1        # Distance from robot center to IR sensor (in meters)

# Units are in CM
MAX_IR_DIST_CM = 63        # Ignore readings further than 80cm
MIN_IR_DIST_CM = 3         # Ignore readings closer than 5cm

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
    
    # --- NEW: Draw Heading Arrow ---
    # We initialize it at (0,0) pointing right (dx=0.2, dy=0)
    # The 'zorder=5' ensures it draws on top of the map
    arrow = ax.arrow(0, 0, 0.1, 0, head_width=0.05, head_length=0.1, fc='blue', ec='blue', zorder=5, label="Heading")

    ax.set_title("Real-Time IR Grid Map")
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
            ir_cm = state['ir']

            # Filter bad sensor data
            if MIN_IR_DIST_CM < ir_cm < MAX_IR_DIST_CM:
                dist_m = (ir_cm / 100.0) + SENSOR_OFFSET
                
                obs_x = rx + dist_m * np.cos(theta)
                obs_y = ry + dist_m * np.sin(theta)

                # Update Grid
                gx, gy = world_to_grid(obs_x, obs_y, center_idx)
                
                if 0 <= gx < grid_cells and 0 <= gy < grid_cells:
                    # Increment "hit" count
                    grid_map[gy, gx] += 3
                    # Cap at 10 to keep the image contrast stable
                    if grid_map[gy, gx] > 10: grid_map[gy, gx] = 10

            # 5. Update Visualization
            robot_plot.set_data([rx], [ry]) 
            img.set_data(grid_map)
            
            # --- NEW: Update Heading Arrow ---
            # Remove the old arrow (arrows cannot be easily updated like lines, they must be redrawn)
            arrow.remove()
            
            # Calculate arrow direction vector (length 0.2 meters)
            arrow_len = 0.1
            dx = arrow_len * np.cos(theta)
            dy = arrow_len * np.sin(theta)
            
            # Draw new arrow at current position
            arrow = ax.arrow(rx, ry, dx, dy, head_width=0.05, head_length=0.1, fc='blue', ec='blue', zorder=5)

            # Use pause to let the GUI event loop run
            plt.pause(0.01) 

    except KeyboardInterrupt:
        print("\nMapping Stopped.")
        # save_map(grid_map)
        # print("-" * 40)

    finally:
        robot.close()
        plt.close('all')

if __name__ == "__main__":
    main()
