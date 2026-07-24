import sys
import os
import time
import numpy as np
import matplotlib.pyplot as plt

# Add parent directory to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from Driver.lib.robot_client import RobotClient
from Nav.lib.map_client import MapClient

def main():
    # 1. Initialize Clients
    robot = RobotClient(host="127.0.0.1")
    map_client = MapClient(host="127.0.0.1")

    print("Viewer: Waiting for first map packet...")
    
    # 2. Wait for Initial Map Data (to determine size/resolution)
    grid = None
    meta = None
    while grid is None:
        update = map_client.get_latest_map()
        if update:
            grid, meta = update
        time.sleep(0.1)

    print(f"Map Initialized. Size: {meta['size']}m x {meta['size']}m")

    # 3. Setup Matplotlib Figure
    plt.ion() # Interactive Mode On
    fig, ax = plt.subplots(figsize=(8, 8))
    
    # Calculate Plot Extents (e.g., -2m to +2m)
    # The map server centers (0,0) in the middle of the array
    half_size = meta['size'] / 2.0
    extent = [-half_size, half_size, -half_size, half_size]
    
    # Draw Initial Map
    # origin='lower' puts (0,0) index at bottom-left corner
    # cmap='Greys' makes high values (100) black and low values (0) white
    img = ax.imshow(grid, cmap='Greys', origin='lower', extent=extent, vmin=0, vmax=100)
    
    # Draw Robot Marker
    robot_dot, = ax.plot([], [], 'bo', markersize=8, label="Robot", zorder=10)
    arrow = None

    # Styling
    ax.set_title("Live Map Monitor")
    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Y (meters)")
    ax.grid(True, linestyle='--', alpha=0.3)
    ax.legend(loc='upper right')
    
    # Fix the axes limits so they don't jump around
    ax.set_xlim(-half_size, half_size)
    ax.set_ylim(-half_size, half_size)

    try:
        while True:
            # --- A. Update Map ---
            update = map_client.get_latest_map()
            if update:
                new_grid, _ = update
                img.set_data(new_grid)
            
            # --- B. Update Robot ---
            state = robot.get_state()
            if state:
                rx, ry = state['x'], state['y']
                theta = state['theta']
                
                # Update Dot
                robot_dot.set_data([rx], [ry])
                
                # Update Heading Arrow
                if arrow: arrow.remove()
                arrow_len = 0.2
                dx = arrow_len * np.cos(theta)
                dy = arrow_len * np.sin(theta)
                arrow = ax.arrow(rx, ry, dx, dy, head_width=0.05, fc='blue', ec='blue', zorder=10)

            # Render
            plt.pause(0.1)

    except KeyboardInterrupt:
        print("Viewer Closed.")
    finally:
        robot.close()
        map_client.close()
        plt.close()

if __name__ == "__main__":
    main()