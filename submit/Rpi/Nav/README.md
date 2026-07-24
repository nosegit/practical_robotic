# Topological Mapping & Navigation

This module implements a **Topological Graph Mapper** for autonomous exploration and pathfinding. Unlike grid-based maps (SLAM), this system builds a sparse graph of **Nodes** (junctions) and **Edges** (paths), making it lightweight and efficient for navigation.

It includes a fully simulated environment to test **Depth First Search (DFS)** exploration and **Breadth First Search (BFS)** pathfinding without physical hardware.

## 📁 File Structure

### Core Logic

| File | Role | Description |
| --- | --- | --- |
| lib/graph_mapper.py | Mapper | The brain of the system. Handles node creation, loop closure detection, and decision-making for autonomous exploration. |
| lib/graph_solver.py | Solver | Implements BFS (Breadth-First Search) to calculate the shortest path (optimal route) between the Start Node and Goal Node. |
| lib/graph_visualizer.py | UI | A dedicated Matplotlib class that visualizes the ground truth maze, the robot's generated graph, and the calculated paths in real-time. |

### Simulation & Data

| File | Role | Description |
| --- | --- | --- |
| demo_run.py | Runner | The main simulation script. It mocks the robot hardware, runs the exploration, and calls the solver upon completion. |
| lib/maze_generator.py | Data | A procedural generator that creates complex 10x10 mazes (100 nodes) with loops and dead ends to stress-test the algorithms. |

### Real Robot

| File | Role | Description |
| --- | --- | --- |
| run.py | Runner | The main script. It run on real hardware on robot, runs the exploration, and calls the solver upon completion. |


## 🚀 How to Run

### 1\. Run the Simulation

To watch the robot explore, map, and solve a 36-node maze:

    python demo_run.py

### 2\. Run on Real Robot

**make sure there nodes are running**
- Driver/robot_service.py
- Vision/camere_server_vid.py

Run robot explorlation on real robot:

    python run.py
    

**What to expect:**

1.  **Exploration (Live Animation):** The robot (Red Star) moves through the maze.
    
    *   **Blue Dots:** Nodes discovered.
        
    *   **Orange Line:** The path the robot has physically traveled (DFS History).
        
    *   **Green Dashes:** Unexplored frontiers ("Unknown" paths).
        
2.  **Completion:** Once the target is found or the map is fully explored.
    
3.  **Solution:** A **Bright Green Line** will appear, showing the optimal BFS path from Start to Finish.
    

## 🧠 Algorithms Explained

### 1\. Topological Mapping (The Graph)

The mapping logic assumes a corridor-based environment.

*   **Scan Strategy:** The robot moves in a chosen direction **until it hits a wall**. It does not stop at the first junction; instead, it scans the entire length of the corridor, identifying multiple junctions along the way.
    
*   **Node Creation:** As it passes openings (left/right) or reaches corners, it creates `MazeNode` objects at those coordinates.
    
*   **Loop Closure:** Before creating a node, the system checks if an existing node is within `MERGE_RADIUS`. If yes, it links to the existing node instead of creating a duplicate.
    

### 2\. Depth First Search (DFS) - Exploration

The robot uses DFS to exhaustively explore the environment.

*   **Strategy:** "Drive until Wall, then Recursively Expand."
    
*   Behavior: 1. Pick a heading.
    
    2\. Drive until the wall, adding all found nodes to the current stack.
    
    3\. Recursively explore the "Unknown" neighbors of the furthest node first.
    
    4\. Backtrack when a dead end is reached.
    

### 3\. Breadth First Search (BFS) - Navigation

Once the map is built, we need the fastest way home or to the goal.

*   **Strategy:** "Expand outwards in layers."
    
*   **Result:** It guarantees the path with the fewest number of node hops. This is visualized as the **Green Solution Path** at the end of the demo.
    

## 📊 Visualization Key

When the simulation runs, the graph visualizer displays the following layers:

*   **Grey Network (Background):** The "Ground Truth" (God's view of the actual maze).
    
*   **Blue Dots/Lines:** The robot's internal map (what it has discovered so far).
    
*   **Orange Highlight:** The history of where the robot has driven.
    
*   **Green Highlight:** The computed shortest path solution.