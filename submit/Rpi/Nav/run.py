from lib.graph_mapper import *
from lib.graph_solver import *
from lib.graph_visualizer import *

''' Autonomous Depth-First Search (DFS) Exploration Script 
    1. Initialize GraphMapper and Start Node
    2. reset robot pose and orientation
    3. Start DFS Exploration from Start Node
    4. Run BFS to find shortest path back to Start Node
    5. Traverse back to Start Node after full exploration
    6. Visualize the mapped graph
'''

if __name__ == "__main__":

    # 1. Initialize GraphMapper and Start Node
    mapper = GraphMapper()
    start_node = mapper.create_node(0, 0)   # Initialize Start Node (Node 1) at (0,0)
    start_node.neighbors['N'] = "Unknown"   # Manually define openings for the start node if known, 
    
    print("Starting Autonomous DFS Exploration...")
    try:
        
        # 2. reset robot pose and orientation
        mapper.robot.reset_pose()               # reset robot pose to (0,0)
        time.sleep(1.0)
        mapper.robot.set_theta(90)              # set robot orientation to 90 degrees
        time.sleep(0.5)

        mapper.traveled_path.append(start_node)  # append start node to traveled path

        # 3. Start DFS Exploration
        mapper.run_dfs(start_node)
        print("MAZE FULLY EXPLORED.")

        # 4. Run BFS to find shortest path back to Start Node
        print("\n--- CALCULATING SHORTEST PATH BACK TO START (BFS) ---")
        solver = GraphSolver(mapper.all_nodes)
        bfs_start = mapper.all_nodes[0]         # Start node (Node 1)
        bfs_goal = mapper.last_visited_node     # Current position of robot after DFS

        bfs_path = solver.find_shortest_path_bfs(bfs_start, bfs_goal)
        bfs_path_ids = [n.id for n in bfs_path]
        print(f"BFS Path Back to Start: {bfs_path_ids}")

        # 4. Traverse back to start
        print("TRAVERSE BACK")
        reverse_path = bfs_path[::-1] # Reverse the BFS path for backtracking
        mapper.traverse_back(reverse_path)

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