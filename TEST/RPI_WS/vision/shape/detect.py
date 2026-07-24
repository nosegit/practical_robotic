import picamera2
import cv2
import numpy as np
import time

def create_color_mask(hsv_image, color_name, color_ranges):
    """
    Creates a clean, binary mask for a specified color.
    
    Args:
        hsv_image (np.array): The input image in HSV color space.
        color_name (str): The name of the color to mask (e.g., "Red").
        color_ranges (dict): The dictionary containing all color limits.

    Returns:
        np.array: A 2D binary mask.
    """
    
    if color_name not in color_ranges:
        print(f"Warning: Color '{color_name}' not defined.")
        return np.zeros(hsv_image.shape[:2], dtype=np.uint8)
        
    limits = color_ranges[color_name]
    
    # Create the primary mask
    mask = cv2.inRange(hsv_image, limits['lower1'], limits['upper1'])
    
    # Handle the special case for "Red"
    if color_name == 'RED':
        mask2 = cv2.inRange(hsv_image, limits['lower2'], limits['upper2'])
        mask = cv2.add(mask, mask2)
        
    # Clean the mask with morphology
    kernel = np.ones((5,5), np.uint8)
    # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
    return mask

def detect_shapes_from_mask(mask, color_name):
    """
    Finds all shapes in a given binary mask and returns their type and location.
    
    Args:
        mask (np.array): The 2D (binary) mask of a single color.
        color_name (str): The name of the color being detected (e.g., "Red").

    Returns:
        list: A list of dictionaries, one for each shape found.
    """
    
    shape_data = [] # This will be our output list
    
    # Find all contours in the provided mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Loop over every contour found
    for c in contours:
        if cv2.contourArea(c) < 100: # Ignore noise
            continue
            
        # --- Get Shape Name ---
        peri = cv2.arcLength(c, True)
        epsilon = 0.025 * peri # Epsilon is key for star detection
        approx = cv2.approxPolyDP(c, epsilon, True)
        vertices = len(approx)

        shape = "Unknown"
        if vertices == 3:
            shape = "Triangle"
        elif vertices == 4:
            shape = "Rectangle"
        elif vertices == 10: # 10 vertices for the star
            shape = "Star"
        elif vertices == 8:
            shape = "Circle"
            
        # --- Get Location ---
        (x, y, w, h) = cv2.boundingRect(c)
        location = (x, y, w, h)
        
        # --- Store the data ---
        shape_data.append({
            'shape': shape,
            'color': color_name,
            'location': location
        })
            
    # if (shape == "Unknown") :
    #     print(f"color:{color_name}, peri:{peri}, epsilon:{epsilon}, approx:{approx}, vertices:{vertices}")
    return shape_data


# initialize color mask range in hue (hue[0,179], saturation[0,255], value)
color_range = {
    'RED'       : { 'lower1': (0, 100, 100),    'upper1': (10, 255, 255),   'lower2': (170, 100, 100),  'upper2': (179, 255, 255)},
    'GREEN'     : { 'lower1': (40, 100, 100),   'upper1': (80, 255, 255) },
    'BLUE'      : { 'lower1': (80,190,20),      'upper1': (120,255,255) },
    'PURPLE'    : { 'lower1': (130, 100, 100),  'upper1': (165, 255, 255) }
}

# Initialise the camera and create a reference to it
camera = picamera2.Picamera2()
config = camera.create_preview_configuration(main={'format': 'RGB888', 'size' : (640,480)})
camera.configure(config)
camera.start()

# Allow the camera time to warm up
time.sleep(0.1)


if __name__ == "__main__" :

    while True:
        image = camera.capture_array("main")

        hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)

        all_shapes = []
        # loop each color
        for color in color_range.keys():
            # create mask
            mask = create_color_mask(hsv,color,color_range)

            # detect shape from mask
            shape_data = detect_shapes_from_mask(mask,color)

            all_shapes.extend(shape_data)

        # print("--- Detected Shapes ---")
        for s in all_shapes:
            # print(f"Found: {s['color']} {s['shape']} at {s['location']}")
            (x, y, w, h) = s['location']
            label = f"{s['color']} {s['shape']}"
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 0), 2)
            cv2.putText(image, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0), 1)
        
        cv2.imshow("Shape and Color Detection", image)

        key = cv2.waitKey(1) & 0xFF
        # if the `esc` key was pressed, break from the loop
        if key == 27:
            break   