"""
Autonomous Maze Exploration & Navigation Script
DFS explores entire reachable maze, then BFS finds shortest path back to start.

Workflow:
  1. Initialize mapper, create start node at (0,0), set heading North
  2. Execute DFS to fully explore all reachable areas (doesn't stop at goal)
  3. After full exploration, BFS finds shortest path from last node back to start
  4. Traverse back along optimal path
  5. Visualize complete mapped graph with DFS history and BFS solution

Key: Goal detection (ArUco5 or RED object at y > 1.5m) is logged but DFS continues exploring.
"""

from lib.graph_mapper import GraphMapper
from lib.graph_solver import GraphSolver
from lib.graph_visualizer import GraphVisualizer
from matplotlib import pyplot as plt
import time



if __name__ == "__main__":

    # 1. Initialize GraphMapper and Start Node
    mapper = GraphMapper()
    start_node = mapper.create_node(0, 0)   # Initialize Start Node (Node 1) at (0,0)
    start_node.neighbors['N'] = "Unknown"   # Manually define openings for the start node if known, 
    
    print("Starting Autonomous DFS Exploration...")
    try:
        bfs_path = None

        # 2. reset robot pose and orientation
        mapper.robot.reset_pose()               # reset robot pose to (0,0)
        time.sleep(1.0)
        mapper.robot.set_theta(90)              # set robot orientation to 90 degrees
        time.sleep(0.5)

        mapper.traveled_path.append(start_node)  # append start node to traveled path

        # 3. DFS explores all reachable areas (continues even after goal detected)
        mapper.run_dfs(start_node)
        print("MAZE FULLY EXPLORED.")

        # exit if goal not reached
        if(not mapper.reach_goal):
            print("WARNING: Goal not reached during exploration.")
            exit(0)

        # 4. BFS to find shortest path back to start
        print("\n--- CALCULATING SHORTEST PATH BACK TO START (BFS) ---")
        solver = GraphSolver(mapper.all_nodes)
        bfs_start = mapper.all_nodes[0]         # Start node (Node 1)
        bfs_goal = mapper.last_visited_node     # Current position of robot after DFS

        bfs_path = solver.find_shortest_path_bfs(bfs_start, bfs_goal)
        bfs_path_ids = [n.id for n in bfs_path]
        print(f"BFS Path Back to Start: {bfs_path_ids}")

        # 5. Traverse back
        print("TRAVERSE BACK")
        reverse_path = bfs_path[::-1] # Reverse the BFS path for backtracking
        mapper.traverse_back(reverse_path)

        print(f"DFS path : {[n.id for n in mapper.traveled_path]}")
        print(f"BFS path : {bfs_path_ids}")
        
    except KeyboardInterrupt:
        print("Exploration Stopped.")
    finally:

        # 5. Visualize the mapped graph
        visualizer = GraphVisualizer()
        visualizer.visualize(
            all_nodes=mapper.all_nodes,
            dfs_path=mapper.traveled_path,
            bfs_path=bfs_path,
            title = "Final Mapped Graph After DFS Exploration"
        )
        plt.show()