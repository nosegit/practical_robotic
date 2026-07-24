''' GraphMapper Module
    Implements Graph Mapping and DFS Exploration for a robot in a maze.
    Key Classes:
    - MazeNode: Represents a node in the maze graph.
    - GraphMapper: Manages the graph, robot movement, and exploration logic.

    Key Functions:
    - create_node: Creates a new node in the graph.
    - explore_path: Moves the robot in a direction until a wall, detecting junctions, create new nodes.
    - run_dfs: Performs recursive depth-first search to explore the maze.
'''

import sys
import os
import time
import math
import numpy as np
import matplotlib.pyplot as plt

# Add parent directory to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
from Driver.lib.robot_client import RobotClient
from Vision.lib.camera_client import CameraClient


# --- CONFIGURATION ---
WALL_THRESHOLD_CM   = 5.0   # 5 cm
JUNCTION_SIZE       = 0.11  # meters
FRONT_STOP_DIST     = 9.0   # CM
CRUISE_SPEED        = 0.1   # meter/s (fix)
ROTATE_SPEED        = 0.5   # rad/s (fix)
SENSOR_MAX_CM       = 40.0  # CM
MERGE_RADIUS        = 0.15  # Meters
LOCAL_MERGE_RADIUS  = 0.01  # Meters

class MazeNode:
    def __init__(self, node_id, x, y):
        self.id = node_id
        self.x = x
        self.y = y
        # Pointers to other MazeNode objects
        # 'N' (90), 'E' (0), 'S' (-90), 'W' (180) based on global map
        # Value can be: None (Wall), "Unknown" (Unexplored), or MazeNode object
        self.neighbors = {'N': None, 'E': None, 'S': None, 'W': None}
        self.type = "Unknown" 

    def __repr__(self):
        return f"Node_{self.id}({self.x:.1f}, {self.y:.1f})"

class GraphMapper:
    def __init__(self):
        self.robot = RobotClient(host="127.0.0.1")
        self.camera = CameraClient(host="127.0.0.1")
        self.all_nodes = []
        self.node_counter = 0
        
        # State
        self.last_visited_node = None 
        self.current_heading = 'E'    
        self.traveled_path = []
        self.reach_goal = False
        self.aruco5_found = False
        self.red_object_found = False

        # Stacks for raw detection
        self.left_stack = []
        self.right_stack = []

    def get_opposite_dir(self, direction):
        pairs = {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}
        return pairs.get(direction)

    def get_side_dirs(self, forward_dir):
        if forward_dir == 'N': return ('W', 'E')
        if forward_dir == 'E': return ('N', 'S')
        if forward_dir == 'S': return ('E', 'W')
        if forward_dir == 'W': return ('S', 'N')
        return (None, None)

    def create_node(self, x, y):
        # Check if node already exists nearby (Loop closure)
        for n in self.all_nodes:
            dist = math.sqrt((n.x - x)**2 + (n.y - y)**2)
            if dist < MERGE_RADIUS:
                return n 
        
        # Create new
        self.node_counter += 1
        new_node = MazeNode(self.node_counter, x, y)
        self.all_nodes.append(new_node)
        return new_node

    # --- DETECTION LOGIC ---
    def process_side(self, side_name, curr_dist, prev_dist, x, y, stack):
        '''
        Processes side ultrasonic sensor data to detect junctions.
        Logic:
        - If distance increases by more than WALL_THRESHOLD_CM,
            it indicates an opening (junction start) -> push '(' onto stack.
        - If distance decreases by more than WALL_THRESHOLD_CM,
            it indicates a closing (junction end) -> push ')' onto stack.
        '''

        # print(f"   Side {side_name}: Prev {prev_dist:.1f}cm, Curr {curr_dist:.1f}cm")
        diff = curr_dist - prev_dist
        if diff > WALL_THRESHOLD_CM:
            print(f"   Detected Junction on {side_name} at ({x:.2f}, {y:.2f})")
            print(f"      Dist Change: {prev_dist:.1f}cm -> {curr_dist:.1f}cm (Diff: {diff:.1f}cm)")
            stack.append({'type': '(', 'pos': np.array([x, y])})
        elif diff < -WALL_THRESHOLD_CM:
            print(f"   Detected End of Junction on {side_name} at ({x:.2f}, {y:.2f})")
            print(f"      Dist Change: {prev_dist:.1f}cm -> {curr_dist:.1f}cm (Diff: {diff:.1f}cm)")
            stack.append({'type': ')', 'pos': np.array([x, y])})

    def resolve_raw_junctions(self, stack):
        ''' 
        Resolves raw junction stack into valid junction positions.
        Logic:
        - Scans through the stack for matching '(' and ')' pairs.
        - Junction valid if a pair is found and the distance between them exceeds JUNCTION_SIZE.
            - Calculates midpoint and records as junction. 
        '''

        junctions = []
        i = 0
        while i < len(stack) - 1:

            p1 = stack[i]['pos']
            p2 = stack[i+1]['pos']
            
            junction_width = abs(p1-p2)
            junction_width = max(junction_width[0],junction_width[1])

            if stack[i]['type'] == '(' and stack[i+1]['type'] == ')' and junction_width > JUNCTION_SIZE:
                mid = (p1 + p2) / 2.0
                junctions.append({'x': mid[0], 'y': mid[1]})
                i += 2
            else:
                i += 1
        return junctions

    # --- MOVEMENT PRIMITIVES ---

    def navigate_to_node(self, target_node):
        """
        Smart Move: Calculates vector from CURRENT robot position 
        to the target node's (x,y) and moves there.
        Useful for recentering the robot after a scan stops at a wall.
        """
        print(f"   [Adjusting] Moving to {target_node}")
        
        # 1. Where am I?
        state = self.robot.get_state()
        # Wait for valid state if needed
        while state is None:
            time.sleep(0.01)
            state = self.robot.get_state()
            
        rx, ry = state['x'], state['y']
        
        # 2. Vector to Target
        dx = target_node.x - rx
        dy = target_node.y - ry
        dist = math.sqrt(dx*dx + dy*dy)
        
        # Threshold: If we are within 1cm, don't waste time moving
        if dist < 0.01:
            # print("   Already at node location.")
            return

        target_angle_rad = math.atan2(dy, dx)
        target_angle_deg = math.degrees(target_angle_rad)
        
        # 3. Execute
        self.robot.rotate_to_angle(target_angle_deg, speed=ROTATE_SPEED)
        time.sleep(0.2)
        self.robot.move_distance(dist, max_speed=CRUISE_SPEED)
        time.sleep(0.2)

    def travel_between_nodes(self, from_node, to_node):
        """Physically moves robot from known node to neighbor node."""
        print(f"   [Move] Node {from_node.id} -> Node {to_node.id}")


        def get_dist(start_x, start_y, end_x, end_y): 
            # dx = to_node.x - from_node.x
            # dy = to_node.y - from_node.y
            dx = end_x - start_x
            dy = end_y - start_y
            dist = math.sqrt(dx*dx + dy*dy)
            target_angle_rad = math.atan2(dy, dx)
            target_angle_deg = math.degrees(target_angle_rad)

            return dist, target_angle_deg
        
        dist, target_angle_deg = get_dist(from_node.x, from_node.y, to_node.x, to_node.y )

        if(dist < 0.01):
            # print("   Already at target node.")
            return
        
        # 1. Rotate
        self.robot.rotate_to_angle(target_angle_deg, speed=ROTATE_SPEED)
        time.sleep(0.5)
        
        # 2. Move
        self.robot.move_distance(dist, max_speed=CRUISE_SPEED)

        # 3. Move to target node again to make sure that robot no the target node.
        state = self.robot._wait_for_valid_state()
        dist, target_angle_deg = get_dist(state['x'], state['y'], to_node.x, to_node.y)
        if(dist > 0.02):
            self.navigate_to_node(to_node)

        time.sleep(0.5)

    def explore_path(self, heading_char):
        """
        Moves robot in 'heading_char' direction until wall.
        Detect wall by IR sensor and ArUco tag distance.
        Detect junctions on sides using side ultrasonic sensors.
        Returns list of NEW nodes found in order.
        If Dead End, returns Empty List and moves 
        robot back to start pos.

        state machine :
            1. prepare to explore
                1. set heading
                2. vision check
                    - get aruco distance
                    - check tag 5 and red object
            2. explore forward until wall
                1. move forward
                2. check front wall
                3. process side sensors for junctions
                    1. add '(' for open junction and ')' for close junction
                        - '(' : when ultrasonic distance increase more than threshold
                        - ')' : when ultrasonic distance decrease more than threshold
            3. process side sensors for junctions and create nodes
                1. resolve raw junctions to valid junctions by matching '(' and ')'
                2. merge nearby junctions within certain radius
                3. create nodes for each junction
                4. identify openings for each node
                5. link backwards to previous node
                6. dead end handling
        """

        #====== 1. Prepare to Explore ======
        print(f"\n>>> EXPLORING {heading_char} ...")
        
        target_angles = {'N': 90, 'E': 0, 'S': -90, 'W': 180}
        self.robot.rotate_to_angle(target_angles[heading_char]) 
        time.sleep(0.5)
        
        self.left_stack = []
        self.right_stack = []
        
        if not self.robot._wait_for_valid_state(): return []
        start_state = self.robot._wait_for_valid_state()
        start_x, start_y = start_state['x'], start_state['y']
        
        prev_us_l = min(start_state.get('us_l', SENSOR_MAX_CM), SENSOR_MAX_CM)
        prev_us_r = min(start_state.get('us_r', SENSOR_MAX_CM), SENSOR_MAX_CM)
        
        # get nearest aruco distance that angle is within 45 degree
        markers = self.camera.get_all_marker()
        dist_nearest_tag = 999  
        if markers:
            for i in markers:
                if(abs(i['yaw']) < 45):
                    dist_nearest_tag = min(dist_nearest_tag,i['distance'])
        print(f"distance to tag : {dist_nearest_tag} meters")

        # check aruco 5 and red object
        for i in range(5) :
            if( self.camera.get_marker_pose(marker_id=5) ): 
                self.aruco5_found = True
                print(f"    [INFO] FOUND TAG 5")
                break

        for i in range(5) :
            if( self.camera.is_color(color="RED") ): 
                self.red_object_found = True
                print(f"    [INFO] FOUND RED OBJECT")
                break
        
        #====== 2. Explore Forward Until Wall ======
        self.robot.set_velocity(CRUISE_SPEED, 0.0)
        hit_wall = False
        start_time = time.time()
        try:
            while True:

                state = self.robot.get_state()
                if not state: continue
                
                ir_dist = state.get('ir', 60)
                ir_speed = (ir_dist/(2*FRONT_STOP_DIST)) * CRUISE_SPEED # Slow down when approaching wall

                dx = state['x']- start_x
                dy = state['y'] - start_y
                dist_traveled = math.sqrt(dx*dx + dy*dy)
                # aruco_speed = ((dist_nearest_tag-dist_traveled)/(0.05)) * CRUISE_SPEED
                aruco_speed = 999

                self.robot.set_velocity( min( min(ir_speed, aruco_speed), CRUISE_SPEED), 0.0)



                # Check Front Wall
                if (ir_dist < FRONT_STOP_DIST) or (dist_traveled > (dist_nearest_tag-0.03) ):
                    print("   Wall Hit. Stopping.")
                    hit_wall = True

                    # add some offset to end position to offset node creation
                    offset = FRONT_STOP_DIST + (14.5 if self.current_heading == "N" or self.current_heading == "S" else 10)
                    rx_offset_cm = {'N': 0, 'E': offset, 'S': 0, 'W': -offset}
                    ry_offset_cm = {'N': offset, 'E': 0, 'S': -offset-2, 'W': 0}

                    # force add ")" after hit wall
                    end_rx = rx+(rx_offset_cm[self.current_heading]*0.01)
                    end_ry = ry+(ry_offset_cm[self.current_heading]*0.01)
                    self.process_side("L", 0, WALL_THRESHOLD_CM+1, end_rx, end_ry, self.left_stack) 
                    self.process_side("R", 0, WALL_THRESHOLD_CM+1, end_rx, end_ry, self.right_stack)

                    break
                
                # Process Sides
                rx, ry = state['x'], state['y']
                us_l = min(state.get('us_l', SENSOR_MAX_CM), SENSOR_MAX_CM)
                us_r = min(state.get('us_r', SENSOR_MAX_CM), SENSOR_MAX_CM)
                self.process_side("L", us_l, prev_us_l, rx, ry, self.left_stack)
                self.process_side("R", us_r, prev_us_r, rx, ry, self.right_stack)
                
                prev_us_l = us_l
                prev_us_r = us_r
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            pass
        
        self.robot.stop()
        time.sleep(0.5)
        
        # ====== 3. process side sensors for junctions and create nodes ======
        # --- Process Logic ---
        raw_left = self.resolve_raw_junctions(self.left_stack)
        raw_right = self.resolve_raw_junctions(self.right_stack)
        
        # Merge Logic
        all_raw = [{'src': 'L', 'data': item} for item in raw_left] + \
                  [{'src': 'R', 'data': item} for item in raw_right]
        all_raw.sort(key=lambda item: (item['data']['x']-start_x)**2 + (item['data']['y']-start_y)**2)
        
        merged_candidates = []
        while len(all_raw) > 0:
            current = all_raw.pop(0)
            cluster = [current]
            rem_idx = 0
            while rem_idx < len(all_raw):
                other = all_raw[rem_idx]
                dist = math.sqrt((current['data']['x'] - other['data']['x'])**2 + 
                                 (current['data']['y'] - other['data']['y'])**2)
                if dist < LOCAL_MERGE_RADIUS:
                    cluster.append(all_raw.pop(rem_idx))
                else:
                    rem_idx += 1
            merged_candidates.append(cluster)
            
        # Create Nodes
        prev_node = self.last_visited_node 
        left_dir, right_dir = self.get_side_dirs(heading_char)
        new_chain = []
        
        for cluster in merged_candidates:
            avg_x = sum(c['data']['x'] for c in cluster) / len(cluster)
            avg_y = sum(c['data']['y'] for c in cluster) / len(cluster)
            
            node = self.create_node(avg_x, avg_y)
            new_chain.append(node)
            
            # Identify Openings
            has_left = any(c['src'] == 'L' for c in cluster)
            has_right = any(c['src'] == 'R' for c in cluster)
            
            if has_left: node.neighbors[left_dir] = "Unknown" if not node.neighbors[left_dir] else node.neighbors[left_dir] 
            if has_right: node.neighbors[right_dir] = "Unknown" if not node.neighbors[right_dir] else node.neighbors[right_dir] 
                
            # Link Backwards
            if prev_node:
                prev_node.neighbors[heading_char] = node
                node.neighbors[self.get_opposite_dir(heading_char)] = prev_node
                
            prev_node = node
            
        # --- DEAD END HANDLING ---
        # If no nodes found, we drove into a dead end. We MUST return to start.
        if not new_chain:
            print("   Dead End (No nodes). Returning to start position.")
            # Calculate distance traveled
            end_state = self.robot.get_state()
            dx = end_state['x'] - start_x
            dy = end_state['y'] - start_y
            dist_traveled = math.sqrt(dx*dx + dy*dy)
            
            # Reverse
            self.robot.rotate(180) # Turn around
            self.robot.rotate_to_angle(self.robot.get_snap_angle())
            self.robot.move_distance(dist_traveled, max_speed=CRUISE_SPEED) # Drive back
            # self.robot.rotate(180) # Face original way (optional, but clean)
            # self.robot.rotate_to_angle(self.robot.get_snap_angle())


            
        return new_chain

    # --- DFS ALGORITHM ---
    def run_dfs(self, current_node):
        """
        Recursive Depth First Search.
        1. Checks current node neighbors.
        2. If 'Unknown', explores them.
        3. Backtracks when done.

        Base Case: If Goal Condition met, sets self.reach_goal = True and returns.
        Recursive Case: For each 'Unknown' neighbor:
            a. Explore Path to discover new nodes.
            b. For each new node in chain (from end to start):
                i. Navigate to that node.
                ii. If not already traveled, recurse DFS from there.
                iii. Backtrack to previous node in chain.
            c. Finally, backtrack to current_node.
        """
        print(f"\n=== DFS VISITING: Node {current_node.id} ===")
        self.last_visited_node = current_node # Sync state

        # ====== Base Case : check Goal======
        state = self.robot._wait_for_valid_state()
        if(state['y'] > 1.5 and (self.aruco5_found or self.red_object_found)): # y > 1.65 
            print("======================== Reach Goal ====================")
            self.reach_goal = True
            return
        
        # Priority: N, W, S, E (Standard)
        directions = ['N', 'W', 'S', 'E']

        # ====== Recursive Case : Explore Neighbors ======
        for direction in directions:
            # Check if this direction is unexplored
            if current_node.neighbors[direction] == "Unknown":

                self.current_heading = direction
                # 1. EXPLORE THE UNKNOWN
                print(f"   Exploring Unknown path: {direction} from Node {current_node.id}")
                new_nodes_chain = self.explore_path(direction)
                
                if not new_nodes_chain:
                    # Case A: Dead End (No junctions found, robot returned)
                    print(f"   Marking {direction} as Closed (Dead End).")
                    current_node.neighbors[direction] = None
                
                else:
                    # Case B: Found a chain of nodes (N1 -> N2 -> ... -> Nk)
                    # Robot is physically at Nk (the last one found)
                    
                    # We need to process this chain. 
                    # Important: The DFS recursion must happen from the *Robot's Current Location* (Nk).
                    # Then we backtrack to N(k-1), recurse, etc.
                    
                    chain = new_nodes_chain
                    print(f"   Chain found: {[n.id for n in chain]}")
                    
                    # 2. RECURSE AND BACKTRACK
                    # Iterate backwards from End of chain (where we are) to Start of chain
                    for i in range(len(chain) - 1, -1, -1):
                        node_here = chain[i]

                        self.navigate_to_node(node_here) # Ensure we are at this node

                        # skip already travel node
                        if(node_here in self.traveled_path):
                            print(f"   Node {node_here.id} already traveled, skipping DFS.")
                            continue

                        
                        # A. Run DFS on this node (handles side paths we skipped while driving past)
                        # Note: We are physically HERE now.
                        current_last_visited_node = self.last_visited_node
                        self.traveled_path.append(node_here)
                        self.run_dfs(node_here)

                        if(self.reach_goal):
                            return
                        self.traveled_path.pop()
                        self.last_visited_node = current_last_visited_node
                        self.navigate_to_node(node_here) # Ensure we are at this node

                        # B. Move to previous node in chain (Backtrack step)
                        if i > 0:
                            node_prev = chain[i-1]
                            self.travel_between_nodes(node_here, node_prev)
                        else:
                            # If i==0 (first node in chain), backtrack returns to 'current_node'
                            self.travel_between_nodes(node_here, current_node)


    def traverse_back(self, path : list[MazeNode] ):
        '''
        Traverse back along a given path of nodes.

        1. Increase speed for traversal.
        2. Adjust final node y-position for accurate stopping.
        3. Navigate to each node in the path sequentially.
        4. Restore original speed settings and final node position.
            
        '''  
        global CRUISE_SPEED, ROTATE_SPEED

        # ======= 1. Increase speed for traversal =======
        old_CRUISE_SPEED = CRUISE_SPEED
        old_ROTATE_SPEED = ROTATE_SPEED
        CRUISE_SPEED = 0.15
        ROTATE_SPEED = 0.75
        
        # ======= 2. Adjust final node y-position for accurate stopping =======
        start_node_offset = 0.10
        path[-1].y = path[-1].y + start_node_offset

        self.navigate_to_node(path[0])
        
        # ======= 3. Navigate to each node in the path sequentially =======
        for i in range(len(path)-1):
            current_node = path[i]
            next_node = path[i+1]
            self.travel_between_nodes(current_node, next_node)
        
        # ======= 4. Restore original speed settings and final node position =======
        CRUISE_SPEED = old_CRUISE_SPEED
        ROTATE_SPEED = old_ROTATE_SPEED
        path[-1].y = path[-1].y - start_node_offset

        self.robot.rotate_to_angle(90)
