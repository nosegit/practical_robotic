'''

Demo script to showcase DFS mapping and BFS solving on a predefined maze.
Uses mocked robot and camera clients to simulate movement and sensing.
Instead of run.py, this demo use simpleDemoMapper inherited from GraphMapper.
GraphMapper is main functionality for DFS exploration (maze exploration).
'''

import sys
import time
import math
import matplotlib.pyplot as plt
from unittest.mock import MagicMock

# 1. MOCK DEPENDENCIES
sys.modules['Driver.lib.robot_client'] = MagicMock()
sys.modules['Vision.lib.camera_client'] = MagicMock()

# 2. IMPORTS
from lib.graph_mapper import GraphMapper
from lib.graph_solver import GraphSolver
from lib.graph_visualizer import GraphVisualizer

# 3. DEFINE THE DEMO MAZE (ADJACENCY MAP)
DEMO_MAZE = {
    # --- ENTRY HALL ---
    1: {'x': 0.0, 'y': 0.0,  'neighbors': {'N': 2}},                           
    
    # --- THE LOOP BLOCK ---
    2: {'x': 0.0, 'y': 1.5,  'neighbors': {'S': 1, 'E': 3, 'W': 4}},           
    3: {'x': 1.5, 'y': 1.5,  'neighbors': {'W': 2, 'N': 5}},                   
    4: {'x': -1.5, 'y': 1.5, 'neighbors': {'E': 2, 'N': 6}},                   
    5: {'x': 1.5, 'y': 3.0,  'neighbors': {'S': 3, 'W': 7}},                   
    6: {'x': -1.5, 'y': 3.0, 'neighbors': {'S': 4, 'E': 7}},                   
    
    # --- CENTRAL HUB ---
    7: {'x': 0.0, 'y': 3.0,  'neighbors': {'E': 5, 'W': 6, 'N': 8}},           
    
    # --- UPPER MAZE ---
    8: {'x': 0.0, 'y': 4.5,  'neighbors': {'S': 7, 'W': 9, 'E': 10}},          
    9: {'x': -1.5, 'y': 4.5, 'neighbors': {'E': 8}},                           
    10: {'x': 1.5, 'y': 4.5, 'neighbors': {'W': 8, 'N': 11}},                  
    11: {'x': 1.5, 'y': 6.0, 'neighbors': {'S': 10, 'W': 12}},                 
    12: {'x': 0.0, 'y': 6.0, 'neighbors': {'E': 11, 'W': 13}},                 
    
    # --- GOAL ---
    13: {'x': -1.5, 'y': 6.0, 'neighbors': {'E': 12}, 'finish': True}          
}

from lib.maze_generator import DEMO_MAZE

class SimpleDemoMapper(GraphMapper):
    def __init__(self):
        super().__init__()
        self.robot = MagicMock()
        self.camera = MagicMock()
        self.robot._wait_for_valid_state = self._get_simulated_state
        
        # Instantiate the new visualizer
        self.viz = GraphVisualizer()
        
        self.aruco5_found = False
        self.red_object_found = False

    def navigate_to_node(self, target_node):
        print(f"   [Adjusting] Moving to {target_node}")
        self.update_visuals()

    def _get_simulated_state(self):
        if self.last_visited_node:
            return {'x': self.last_visited_node.x, 'y': self.last_visited_node.y, 'theta': 0.0}
        return {'x': 0.0, 'y': 0.0, 'theta': 0.0}

    def _get_demo_data_by_coord(self, x, y):
        for demo_id, data in DEMO_MAZE.items():
            dist = math.sqrt((data['x'] - x)**2 + (data['y'] - y)**2)
            if dist < 0.1: 
                return demo_id, data
        return None, None

    def _find_existing_node_at(self, x, y):
        for node in self.all_nodes:
            dist = math.sqrt((node.x - x)**2 + (node.y - y)**2)
            if dist < 0.1:
                return node
        return None

    def update_visuals(self):
        """Helper to update the live visualization"""
        status = "SEARCHING..."
        if self.aruco5_found and self.red_object_found:
            status = "TARGET ACQUIRED!"
            
        title = f"Demo: {status} | Nodes: {len(self.all_nodes)}"
        
        self.viz.visualize(
            all_nodes=self.all_nodes,
            gt_maze=DEMO_MAZE,
            dfs_path=self.traveled_path,
            current_node=self.last_visited_node,
            title=title
        )

    def explore_path(self, heading):
        current_node = self.last_visited_node
        curr_demo_id, curr_data = self._get_demo_data_by_coord(current_node.x, current_node.y)
        
        if not curr_data:
            print(f"   [Error] Robot lost at ({current_node.x}, {current_node.y})")
            return []

        next_demo_id = curr_data['neighbors'].get(heading)

        if next_demo_id is None:
            print(f"   [Wall] Hit wall from Node {current_node.id} facing {heading}")
            return []

        target_data = DEMO_MAZE[next_demo_id]
        tx, ty = target_data['x'], target_data['y']
        
        existing_node = self._find_existing_node_at(tx, ty)
        
        if existing_node:
            print(f"   [Scan] Found EXISTING Node {existing_node.id} at ({tx}, {ty})")
            next_node = existing_node
        else:
            next_node = self.create_node(tx, ty)
            print(f"   [Scan] Discovered NEW Node {next_node.id} at ({tx}, {ty})")
            
            opp_heading = self.get_opposite_dir(heading)
            for d in target_data['neighbors']:
                if d != opp_heading:
                    next_node.neighbors[d] = "Unknown"

        if target_data.get('finish', False):
            print(f"\n   !!! MISSION ACCOMPLISHED AT NODE {next_node.id} !!!")
            self.aruco5_found = True
            self.red_object_found = True

        opp_heading = self.get_opposite_dir(heading)
        current_node.neighbors[heading] = next_node
        next_node.neighbors[opp_heading] = current_node

        self.last_visited_node = next_node
        
        # --- UPDATE LIVE VISUALS ---
        self.update_visuals()
        
        remaining_nodes = self.explore_path(heading)
        return [next_node] + remaining_nodes

    def travel_between_nodes(self, from_node, to_node):
        print(f"   >> Navigating: Node {from_node.id} -> Node {to_node.id}")
        self.last_visited_node = to_node
        
        # --- UPDATE LIVE VISUALS ---
        self.update_visuals()
        
        time.sleep(0.01)

    def rotate_to_angle(self, angle): pass
 
if __name__ == "__main__":
    mapper = SimpleDemoMapper()
    
    print("Initializing Map...")
    start_data = DEMO_MAZE[1]
    start_node = mapper.create_node(start_data['x'], start_data['y'])

    for direction in start_data['neighbors']:
        start_node.neighbors[direction] = "Unknown"
    
    mapper.traveled_path.append(start_node)
    mapper.last_visited_node = start_node
    
    # Initial plot
    mapper.update_visuals()
    input("Press Enter to continue...")
    
    # start DFS to expore the maze
    print("\n--- STARTING COMPLEX DFS DEMO ---")
    mapper.run_dfs(start_node)
    print("\n--- MAPPING COMPLETE ---")

    if not mapper.reach_goal:
        print("WARNING: Goal node was not reached during DFS.")
        exit (0)

    # --- RUN BFS SOLVER ---
    print("\n--- CALCULATING SHORTEST PATH (BFS) ---")
    
    solver = GraphSolver(mapper.all_nodes)
    bfs_start = mapper.all_nodes[0] 
    bfs_goal = mapper.last_visited_node
    bfs_path = solver.find_shortest_path_bfs(bfs_start, bfs_goal)
    

    # --- Print Output ---
    dfs_path_ids = [n.id for n in mapper.traveled_path]
    bfs_path_ids = [n.id for n in bfs_path]

    print(f"DFS Traveled Path ({len(dfs_path_ids)} steps): {dfs_path_ids}")
    print(f"BFS Shortest Path ({len(bfs_path_ids)} steps): {bfs_path_ids}")

    # --- FINAL VISUALIZATION ---
    print("Visualizing Final State...")
    
    status = "SEARCHING..."
    if mapper.aruco5_found and mapper.red_object_found:
        status = "TARGET ACQUIRED!"
        
    title = f"Demo: {status} | Nodes: {len(mapper.all_nodes)}"
    
    mapper.viz.visualize(
        all_nodes=mapper.all_nodes,
        gt_maze=DEMO_MAZE,
        dfs_path=mapper.traveled_path,
        bfs_path=bfs_path,
        current_node=mapper.last_visited_node,
        title=title
    )
        
    plt.show()