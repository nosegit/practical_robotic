DEMO_MAZE = {
    # --- ENTRY HALL ---
    1: {'x': 0.0, 'y': 0.0,  'neighbors': {'N': 2}},                           # Start
    
    # --- THE LOOP BLOCK (Nodes 2, 3, 5, 7, 6, 4) ---
    # Node 2 is a T-Junction (Wall to North), forcing a turn.
    2: {'x': 0.0, 'y': 1.5,  'neighbors': {'S': 1, 'E': 3, 'W': 4}},           # T-Junction
    3: {'x': 1.5, 'y': 1.5,  'neighbors': {'W': 2, 'N': 5}},                   # Corner
    4: {'x': -1.5, 'y': 1.5, 'neighbors': {'E': 2, 'N': 6}},                   # Corner
    5: {'x': 1.5, 'y': 3.0,  'neighbors': {'S': 3, 'W': 7, 'N':10}},                   # Corner
    6: {'x': -1.5, 'y': 3.0, 'neighbors': {'S': 4, 'E': 7}},                   # Corner
    
    # --- CENTRAL HUB ---
    # Node 7 connects the loop to the upper section
    7: {'x': 0.0, 'y': 3.0,  'neighbors': {'E': 5, 'W': 6, 'N': 8}},           # 4-Way Junction
    
    # --- UPPER MAZE (DEAD ENDS & WINDING PATH) ---
    8: {'x': 0.0, 'y': 4.5,  'neighbors': {'S': 7, 'W': 9, 'E': 10, 'N':12}},          # Junction
    9: {'x': -1.5, 'y': 4.5, 'neighbors': {'E': 8, 'S':6}},                           # Dead End (Trap)
    10: {'x': 1.5, 'y': 4.5, 'neighbors': {'W': 8, }},                  # Corner
    11: {'x': 1.5, 'y': 6.0, 'neighbors': {'S': 10, 'W': 12}},                 # Corner
    12: {'x': 0.0, 'y': 6.0, 'neighbors': {'E': 11, 'W': 13}},                 # Pre-Finish
    
    # --- GOAL ---
    13: {'x': -1.5, 'y': 6.0, 'neighbors': {'E': 12}, 'finish': True},          # FINISH NODE

}