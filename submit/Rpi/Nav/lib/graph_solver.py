"""
GraphSolver: BFS-based pathfinding for finding shortest path in discovered maze graph.
Implements standard breadth-first search algorithm to guarantee shortest path by node count.
Handles graph navigation with validation of node connections and unexplored frontiers.

Key feature: Ignores "Unknown" and None neighbors (unexplored/walls) to ensure valid paths.
"""


from collections import deque
from lib.graph_mapper import MazeNode


class GraphSolver:
    def __init__(self, all_nodes: list[MazeNode]):
        """
        Initialize with the complete graph of nodes.
        :param all_nodes: List of MazeNode objects
        """
        self.all_nodes = all_nodes

    def find_shortest_path_bfs(self, start_node, goal_node):
        """
        Perform breadth-first search to find shortest path by node count.
        Uses standard BFS algorithm with queue-based exploration.
        
        Algorithm:
          1. Initialize queue with (current_node, path_to_here) tuples
          2. Maintain visited set by node IDs to avoid cycles
          3. For each node, explore all valid neighbors (skip walls/unknown)
          4. Return full path when goal is reached
          5. Return empty list if no path exists
        
        Node neighbor validation:
          - Skip None (wall)
          - Skip "Unknown" (unexplored frontier)
          - Only process actual MazeNode objects
        """
        if not start_node or not goal_node:
            print("   [Solver] Error: Start or Goal node invalid.")
            return []

        # Queue stores tuples: (current_node, [path_nodes_so_far])
        # Storing full path enables direct reconstruction without parent pointers
        queue = deque([(start_node, [start_node])])
        visited = {start_node.id}

        print(f"   [Solver] Searching from Node {start_node.id} to Node {goal_node.id}...")

        while queue:
            current, path = queue.popleft()

            # Goal check: terminate immediately upon reaching goal
            if current == goal_node:
                return path

            # Explore all four cardinal directions (N, E, S, W)
            for direction, neighbor in current.neighbors.items():
                
                # Validation checks:
                # 1. neighbor is not None (wall)
                # 2. neighbor is not "Unknown" (unexplored frontier)
                # 3. neighbor has valid node structure (has 'id' attribute)
                # 4. neighbor hasn't been visited yet
                if neighbor and neighbor != "Unknown":
                    # Check ID to ensure it's a valid node object
                    if hasattr(neighbor, 'id') and neighbor.id not in visited:
                        visited.add(neighbor.id)
                        
                        # Extend path with new node
                        new_path = list(path)
                        new_path.append(neighbor)
                        queue.append((neighbor, new_path))
        
        # No path found after exhausting all reachable nodes
        print("   [Solver] No path found.")
        return []