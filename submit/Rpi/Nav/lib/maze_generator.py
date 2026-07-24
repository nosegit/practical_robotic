import random
import sys

# Increase recursion depth just in case for larger mazes
sys.setrecursionlimit(2000)

def generate_large_maze(rows=10, cols=10, step=1.5):
    """
    Generates a dictionary representing a maze with 'rows * cols' nodes.
    Uses Recursive Backtracker for the spanning tree + random edges for loops.
    """
    maze = {}
    
    # 1. Initialize Grid of Nodes (Disconnected)
    # Map (r, c) -> ID
    grid_map = {}
    node_id_counter = 1
    
    for r in range(rows):
        for c in range(cols):
            # ID, x, y
            nid = node_id_counter
            grid_map[(r, c)] = nid
            
            maze[nid] = {
                'x': float(c * step),
                'y': float(r * step),
                'neighbors': {} # Starts with no connections (Walls everywhere)
            }
            node_id_counter += 1

    # 2. Recursive Backtracker to carve a perfect maze (Spanning Tree)
    visited = set()
    
    # Directions: (dr, dc, Name, OppositeName)
    # y is row (North/South), x is col (East/West)
    # N: r+1, S: r-1, E: c+1, W: c-1
    directions = [
        (1, 0, 'N', 'S'),
        (-1, 0, 'S', 'N'),
        (0, 1, 'E', 'W'),
        (0, -1, 'W', 'E')
    ]

    def get_valid_neighbors(r, c):
        ns = []
        for dr, dc, d_name, d_opp in directions:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols:
                ns.append((nr, nc, d_name, d_opp))
        return ns

    def carve(r, c):
        visited.add((r, c))
        curr_id = grid_map[(r, c)]
        
        # Get neighbors and shuffle for randomness
        neighbors = get_valid_neighbors(r, c)
        random.shuffle(neighbors)
        
        for nr, nc, d_name, d_opp in neighbors:
            if (nr, nc) not in visited:
                next_id = grid_map[(nr, nc)]
                
                # Link Graph (Carve Wall)
                maze[curr_id]['neighbors'][d_name] = next_id
                maze[next_id]['neighbors'][d_opp] = curr_id
                
                # Recurse
                carve(nr, nc)

    # Start carving from (0,0)
    carve(0, 0)

    # 3. Add Loops (Randomly remove walls between existing nodes)
    # This makes the maze imperfect (multiple paths), which is good for SLAM testing.
    # We try to add about 15% extra edges
    extra_edges_count = int(rows * cols * 0.15)
    added_count = 0
    attempts = 0
    
    while added_count < extra_edges_count and attempts < 1000:
        attempts += 1
        r = random.randint(0, rows - 1)
        c = random.randint(0, cols - 1)
        curr_id = grid_map[(r, c)]
        
        # Pick a random direction
        dr, dc, d_name, d_opp = random.choice(directions)
        nr, nc = r + dr, c + dc
        
        if 0 <= nr < rows and 0 <= nc < cols:
            next_id = grid_map[(nr, nc)]
            
            # If not already connected
            if d_name not in maze[curr_id]['neighbors']:
                maze[curr_id]['neighbors'][d_name] = next_id
                maze[next_id]['neighbors'][d_opp] = curr_id
                added_count += 1

    # 4. Set Finish Node (Top Right corner)
    finish_id = grid_map[(rows-1, cols-1)]
    maze[finish_id]['finish'] = True

    print(f"Generated {len(maze)} nodes with {added_count} loop closures.")
    return maze

# Generate the maze constant to be imported
DEMO_MAZE = generate_large_maze(rows=10, cols=10)