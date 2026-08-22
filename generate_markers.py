import cv2
import cv2.aruco as aruco
import os

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
border = 40

for i in range(1, 9):
    # Generate 400x400 image
    img = aruco.generateImageMarker(aruco_dict, i, 400)
    
    # Add white border
    img_with_border = cv2.copyMakeBorder(img, border, border, border, border, cv2.BORDER_CONSTANT, value=[255, 255, 255])
    
    path = os.path.expanduser(f'~/PX4-Autopilot/Tools/simulation/gz/models/corridor_marker_{i}')
    os.makedirs(path, exist_ok=True)
    
    filepath = os.path.join(path, 'marker.png')
    cv2.imwrite(filepath, img_with_border)
    print(f'Generated {filepath} for ArUco ID {i}')

print('All 8 unique markers generated successfully!')
