from collections import deque
import matplotlib.pyplot as plt
from lib.graph_mapper import MazeNode


class GraphSolver:
    def __init__(self, all_nodes):
        """
        Initialize with the complete graph of nodes.
        :param all_nodes: List of MazeNode objects
        """
        self.all_nodes = all_nodes

    def find_shortest_path_bfs(self, start_node, goal_node):
        """
        Performs Breadth-First Search to find the shortest path (by node count)
        from start_node to goal_node using the discovered connections.
        
        :return: List of MazeNode objects representing the path, or [] if none found.
        """
        if not start_node or not goal_node:
            print("   [Solver] Error: Start or Goal node invalid.")
            return []

        # Queue stores tuples of (current_node, path_to_here_list)
        # We store the full path in the queue to reconstruct it easily
        queue = deque([(start_node, [start_node])])
        visited = {start_node.id}

        print(f"   [Solver] Searching from Node {start_node.id} to Node {goal_node.id}...")

        while queue:
            current, path = queue.popleft()

            # Check if we reached the goal
            if current == goal_node:
                return path

            # Explore neighbors
            # The structure of neighbors is { 'N': NodeObj, 'E': None, ... }
            for direction, neighbor in current.neighbors.items():
                
                # Validation:
                # 1. Must be a valid object (not None/Wall)
                # 2. Must not be the string "Unknown" (unexplored frontier)
                # 3. Must not be visited yet
                if neighbor and neighbor != "Unknown":
                    # Check ID to ensure it's a valid node object
                    if hasattr(neighbor, 'id') and neighbor.id not in visited:
                        visited.add(neighbor.id)
                        
                        new_path = list(path)
                        new_path.append(neighbor)
                        queue.append((neighbor, new_path))
        
        print("   [Solver] No path found.")
        return []