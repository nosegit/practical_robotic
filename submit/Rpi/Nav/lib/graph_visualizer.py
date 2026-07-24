'''
Graph Visualizer Module
Provides functionality to visualize the robot's mapped graph.
'''

import matplotlib.pyplot as plt

class GraphVisualizer:
    """
    Dedicated class for visualizing the robot's mapping progress,
    ground truth comparison, and pathfinding results.
    """
    def __init__(self):
        pass

    def visualize(self, 
                  all_nodes=None, 
                  gt_maze=None, 
                  dfs_path=None, 
                  bfs_path=None, 
                  current_node=None,
                  title="Graph Visualization"):
        """
        Main plotting function. Layers data from background to foreground.
        
        :param all_nodes: List of discovered MazeNode objects (The Robot's Map).
        :param gt_maze: Dictionary of the Ground Truth Maze (DEMO_MAZE).
        :param dfs_path: List of MazeNodes representing the robot's travel history.
        :param bfs_path: List of MazeNodes representing the calculated shortest path.
        :param current_node: The MazeNode object where the robot currently is.
        :param title: String title for the plot.
        """

        print("\n--- GRAPH STATE ---")
        for n in all_nodes:
            conns = []
            for d, neighbor in n.neighbors.items():
                if neighbor == "Unknown": val = "?"
                elif neighbor is None: val = "X"
                else: val = f"N{neighbor.id}"
                conns.append(f"{d}:{val}")
            print(f"Node {n.id}: {', '.join(conns)}")

        plt.clf()
        
        # --- LAYER 1: GROUND TRUTH (Background) ---
        if gt_maze:
            self._plot_ground_truth(gt_maze)

        # --- LAYER 2: DFS TRAVELED PATH (History Highlight) ---
        if dfs_path and len(dfs_path) > 1:
            px = [n.x for n in dfs_path]
            py = [n.y for n in dfs_path]
            # Thick semi-transparent orange line
            plt.plot(px, py, color='orange', linewidth=5, alpha=0.7, label='DFS History', zorder=2.0)

        # --- LAYER 3: BFS SOLUTION PATH (Optimal Highlight) ---
        if bfs_path and len(bfs_path) > 1:
            sx = [n.x for n in bfs_path]
            sy = [n.y for n in bfs_path]
            # Bright green line
            plt.plot(sx, sy, color='#00FF00', linewidth=6, alpha=0.7, label='BFS Solution', zorder=1.8)
            # Markers for start/end of BFS
            plt.plot(bfs_path[0].x, bfs_path[0].y, 'g^', markersize=12, zorder=5) # Start
            plt.plot(bfs_path[-1].x, bfs_path[-1].y, 'r*', markersize=12, zorder=5) # Goal

        # --- LAYER 4: DISCOVERED GRAPH (Foreground) ---
        if all_nodes:
            self._plot_discovered_nodes(all_nodes)

        # --- LAYER 5: ROBOT POSITION ---
        if current_node:
            plt.plot(current_node.x, current_node.y, 'r*', markersize=15, label='Robot', zorder=6)

        # --- DECORATIONS ---
        plt.axis('equal')
        plt.grid(True, linestyle=':', alpha=0.6)
        plt.title(title)
        plt.legend(loc='lower right')
        plt.draw()
        plt.pause(0.1)

    def _plot_ground_truth(self, gt_maze):
        """Helper to plot the Demo Maze / Ground Truth"""
        for demo_id, data in gt_maze.items():
            # Node color: Red if finish, Light Gray otherwise
            color = '#FF9999' if data.get('finish') else '#E0E0E0'
            size = 18 if data.get('finish') else 14
            
            # Plot Node
            plt.plot(data['x'], data['y'], 'o', color=color, markersize=size, zorder=1)
            
            # Label Ground Truth Node
            plt.text(data['x'] - 0.2, data['y'] - 0.2, f"GT:{demo_id}", 
                     color='gray', fontsize=8, fontweight='bold', zorder=2)
            
            # Plot Edges (Neighbors)
            for heading, neighbor_id in data['neighbors'].items():
                if neighbor_id in gt_maze:
                    n_data = gt_maze[neighbor_id]
                    plt.plot([data['x'], n_data['x']], [data['y'], n_data['y']], 
                             color='#E0E0E0', linestyle='-', linewidth=4, zorder=0)

    def _plot_discovered_nodes(self, nodes):
        """Helper to plot what the robot has actually mapped"""
        for n in nodes:
            # Plot Node (Blue Dot)
            plt.plot(n.x, n.y, 'bo', markersize=6, zorder=3)
            # Label Node ID
            plt.text(n.x+0.05, n.y+0.05, f"ID:{n.id}", 
                     color='blue', fontsize=9, fontweight='bold', zorder=4)
            
            # Plot Edges
            for heading, neighbor in n.neighbors.items():
                # Case A: Connected to another Node
                if neighbor and neighbor != "Unknown":
                    # Verify it's an object with coordinates
                    if hasattr(neighbor, 'x'):
                        plt.plot([n.x, neighbor.x], [n.y, neighbor.y], 'b-', linewidth=1.5, zorder=2)
                
                # Case B: Unexplored Frontier ("Unknown")
                elif neighbor == "Unknown":
                    # Draw a small green dashed stub
                    dx, dy = 0, 0
                    stub = 0.5
                    if heading == 'N': dy = stub
                    elif heading == 'S': dy = -stub
                    elif heading == 'E': dx = stub
                    elif heading == 'W': dx = -stub
                    plt.plot([n.x, n.x+dx], [n.y, n.y+dy], 'g--', linewidth=1, zorder=2)

