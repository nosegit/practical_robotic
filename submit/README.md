# Autonomous Mobile Robot - Maze Exploration & Navigation

A complete robotic system for autonomous maze exploration using Depth-First Search (DFS) pathfinding and optimal path calculation via BFS. The robot autonomously maps unknown maze environments, detects goal markers, and returns via the shortest path.

## Project Overview

**Objective:** Robot autonomously explores an unknown maze, creates a spatial graph map, detects goal locations (ArUco marker 5 or RED object), then navigates back to start via optimal path.

**Key Technologies:**
- ZeroMQ (ZMQ) for inter-process communication
- OpenCV for vision processing (ArUco markers, color detection)
- Raspberry Pi with PiCamera2
- Ultrasonic and IR sensors for obstacle detection
- Depth-First Search (DFS) for exhaustive exploration
- Breadth-First Search (BFS) for shortest path calculation

---

## Directory Structure

```
submit/
├── README.md                    # This file
├── Rpi/
│   ├── requirements.txt         # Python dependencies
│   ├── Driver/                  # Low-level hardware control
│   │   ├── lib/
│   │   │   ├── robot_driver.py         # Serial communication with robot firmware
│   │   │   └── robot_client.py         # High-level robot control client
│   │   └── robot_service.py            # ZMQ service bridging driver ↔ clients
│   │
│   ├── Vision/                  # Computer vision & marker detection
│   │   ├── lib/
│   │   │   └── camera_client.py        # Vision data client (subscribes to camera server)
│   │   ├── camera_server_vid.py        # Main camera server (ArUco + color detection)
│   │   ├── camera_viewer.py            # Real-time video stream viewer
│   │   └── lib/camera_calibration.py   # Camera intrinsic calibration tool
│   │
│   ├── Test/                    # Script for testing. ex, IR vs Aruco plot. 
│   │
│   └── Nav/                     # Navigation & maze exploration
│       ├── run.py                      # Main autonomous exploration script
│       └── lib/
│           ├── graph_mapper.py         # DFS exploration & maze graph construction
│           ├── graph_solver.py         # BFS pathfinding for shortest path
│           └── graph_visualizer.py     # Matplotlib visualization of discovered map
├── Arduino/                     # Arduino files
```

---

## Installation & Setup

### Prerequisites
- Python 3.8 or higher
- Raspberry Pi 4/5 with PiCamera2 (for vision system)
- USB connection to robot microcontroller
- Network connection between devices (for remote control)

### Install Dependencies on RPI

```bash
# Navigate to project directory
cd submit/Rpi

# 1. Create the new environment with system access
python -m venv --system-site-packages my_pi_env

# 2. Activate it
source my_pi_env/bin/activate

# 3. Install required Python packages
pip install -r requirements.txt
```

### Dependency Versions

| Package | Version | Purpose |
|---------|---------|---------|
| **zmq** (pyzmq) | 27.1.0 | Inter-process communication, ZeroMQ bindings |
| **opencv-python** | 4.11.0.86 | Computer vision, ArUco detection, color filtering |
| **picamera2** | 0.3.21 | Raspberry Pi camera interface (libcamera) |
| **numpy** | 1.24.2 | Numerical computations, array operations |
| **matplotlib** | 3.6.3 | Graph visualization and plotting |
| **pyserial** | 3.5 | Serial communication with robot hardware |

### Critical Version Notes

- **OpenCV 4.11.0.86**: for Aruco PoseEstimation using solvePnP method.

---

## Component Descriptions

### 1. Driver (Rpi/Driver/)

**Purpose:** Low-level hardware communication and robot control.

**Files:**
- **robot_driver.py:** 
  - Handles USB serial communication with robot microcontroller
  - Manages bidirectional packet protocol (command/telemetry)
  - Runs background thread for non-blocking telemetry reading
  - Parses binary packets into robot state (position, velocity, sensor readings)

- **robot_client.py:** 
  - High-level Python API for robot control
  - Velocity commands (linear + angular)
  - Movement primitives: `move_distance()`, `rotate()`, `rotate_to_angle()`
  - Non-blocking state queries via ZMQ telemetry stream

- **robot_service.py:** 
  - Central ZMQ service connecting Driver ↔ Clients
  - Publishes robot telemetry at 50 Hz (port 5555)
  - Receives velocity commands (port 5556)
  - Bridges low-level driver with high-level applications

---

### 2. Vision (Rpi/Vision/)

**Purpose:** Real-time computer vision for marker detection and goal identification.

**Files:**
- **camera_server_vid.py:** 
  - Main vision processing server running on Pi with camera
  - Detects ArUco markers (6x6_250 dictionary) with 3D pose estimation
  - Detects colored shapes (RED, GREEN, BLUE, PURPLE) via HSV thresholding
  - Publishes JSON detection data (port 5557) and JPEG video stream (port 5558)
  - Requires camera calibration for accurate 3D pose

- **camera_client.py:** 
  - Client library for querying vision data
  - Methods: `get_marker_pose()`, `get_closest_shape()`, `is_color()`
  - Automatically drains buffered frames for low-latency latest data

- **camera_viewer.py:** 
  - Standalone viewer displaying live video stream from camera server
  - Manual frame draining for minimal latency
  - Useful for debugging and live monitoring

- **lib/camera_calibration.py:** 
  - Interactive tool for calibrating camera intrinsics
  - Captures chessboard patterns, computes camera matrix + distortion
  - Saves calibration_data.npz for use in 3D pose estimation
  - **Prerequisite:** Run before using vision-based 3D features

---

### 3. Navigation (Rpi/Nav/)

**Purpose:** Autonomous maze exploration and pathfinding.

**Files:**
- **run.py:** 
  - Main entry point for autonomous exploration
  - Initializes mapper, starts DFS, computes shortest path via BFS
  - Traverses back to start along optimal route
  - Visualizes final discovered graph

- **lib/graph_mapper.py:** 
  - **MazeNode:** Graph node representing junction/location with position (x,y) and 4 neighbors (N,E,S,W)
  - **GraphMapper:** Core exploration engine
    - DFS exploration with recursive neighbor discovery
    - Junction detection via ultrasonic sensor analysis (5cm threshold)
    - Smart node merging for loop closure (0.15m merge radius)
    - Dead-end handling and automatic backtracking
    - Goal detection (ArUco marker 5 or RED object at y > 1.5m)
  - **Key Methods:**
    - `explore_path()`: Moves robot forward, detects junctions, creates nodes
    - `run_dfs()`: Recursive DFS to exhaustively explore maze
    - `traverse_back()`: Executes BFS path with optimized speed

- **lib/graph_solver.py:** 
  - **GraphSolver:** BFS pathfinding engine
  - `find_shortest_path_bfs()`: Finds shortest path by node count between any two nodes
  - Validates connections (skips walls/unknown frontiers)
  - Returns full node path for execution

- **lib/graph_visualizer.py:** 
  - **GraphVisualizer:** Matplotlib-based visualization
  - Renders: ground truth maze, DFS history (orange), BFS solution (green), discovered nodes (blue)
  - Shows unexplored frontiers (green dashed stubs)
  - Helpful for debugging mapping errors and verifying pathfinding correctness

---

## System Architecture

### Communication Protocols

| Component | Protocol | Direction | Port | Frequency |
|-----------|----------|-----------|------|-----------|
| robot_service | ZMQ PUB | Telemetry Out | 5555 | 50 Hz |
| robot_service | ZMQ REP | Commands In | 5556 | Variable |
| camera_server | ZMQ PUB | Detection JSON | 5557 | 10 Hz |
| camera_server | ZMQ PUB | Video JPEG | 5558 | 10 Hz |

---

## Running the System

### 1. Start Hardware Driver & Service (on Raspberry Pi)

```bash
cd Rpi/Driver
python robot_service.py
```

### 2. Start Vision Server (on Raspberry Pi with camera)

```bash
cd Rpi/Vision
python camera_server_vid.py
```

### 3. Optional: View Live Camera Feed

```bash
cd Rpi/Vision
python camera_viewer.py
```

### 4. Start Autonomous Exploration (on local machine or Pi)

```bash
cd Rpi/Nav
python run.py
```

Expected output:
```
Starting Autonomous DFS Exploration...
Hardware Driver Started Successfully.
=== DFS VISITING: Node 1 ===
>>> EXPLORING N ...
distance to tag : 2.45 meters
...
MAZE FULLY EXPLORED.
--- CALCULATING SHORTEST PATH BACK TO START (BFS) ---
BFS Path Back to Start: [5, 3, 2, 1]
TRAVERSE BACK
```

---

## Key Concepts

### DFS Exploration Algorithm

1. **Initialize:** Create start node at (0,0), set heading North (90°)
2. **Recursive DFS:** For each node, explore unknown neighbors (priority: N→W→S→E)
3. **Junction Detection:** Move forward while analyzing side ultrasonic sensors
4. **Node Creation:** Merge nearby junctions, create nodes with bidirectional links
5. **Dead-End Handling:** If no junctions found, reverse and backtrack
6. **Goal Detection:** Log goal location but continue exploring all reachable areas
7. **Complete Exploration:** Return when all neighbors exhausted

### BFS Pathfinding

- Finds shortest path (by node count) from any node to any other
- Guarantees optimality for unweighted graphs
- Skips walls (None) and unexplored frontiers ("Unknown")

### Junction Detection

- **Threshold:** 5cm ultrasonic distance change indicates opening/closing
- **Stack-based:** Records '(' for opening, ')' for closing
- **Validation:** Matches pairs and filters by minimum size (11cm)
- **Merging:** Nearby junctions (1cm radius) clustered into single nodes

---

## Configuration Parameters

### Vision (camera_server_vid.py)
- `MARKER_SIZE = 0.065m` : ArUco marker physical size
- `COLOR_RANGES` : HSV thresholds for RED, GREEN, BLUE, PURPLE

### Navigation (graph_mapper.py)
- `WALL_THRESHOLD_CM = 5.0` : Ultrasonic change for junction detection
- `JUNCTION_SIZE = 0.11m` : Minimum junction width
- `FRONT_STOP_DIST = 9.0cm` : IR distance to stop at wall
- `CRUISE_SPEED = 0.1 m/s` : Forward movement speed
- `MERGE_RADIUS = 0.15m` : Loop closure distance

---