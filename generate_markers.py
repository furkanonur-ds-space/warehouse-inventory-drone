"""
Generate the eight ArUco floor marker models placed at the aisle ends.

Each marker gets its own model directory containing the texture, an SDF and a
model.config, so the whole set can be rebuilt from this repository alone. An
earlier version wrote only the texture and left the SDF files to be created by
hand, which meant the world could not be reproduced from a clean checkout.

The texture filename carries the marker id (marker_1.png, marker_2.png, ...).
This matters: Gazebo caches textures by basename, so when all eight models
stored their texture as "marker.png" the cache treated them as one asset and
rendered every marker in the world with the first one loaded. Measured with the
vehicle 0.02 m from marker 7 and 0.06 m from marker 8, the downward camera read
id 1 in both cases, 0 of 15 detections correct. This is the same failure the
box QR textures already hit, and the fix is the same: unique filenames.

The dictionary must stay in step with ARUCO_DICT in warehouse_scanner.py. A
mismatch is silent, since the detector simply never matches.
"""
import os

import cv2
import cv2.aruco as aruco

DICTIONARY = aruco.DICT_4X4_50
MARKER_COUNT = 8
MARKER_PX = 400
BORDER_PX = 40          # white quiet zone, required for reliable detection
MARKER_SIZE_M = 0.4     # side length of the plane in the world

MODELS_DIR = os.path.expanduser('~/PX4-Autopilot/Tools/simulation/gz/models')

MODEL_CONFIG = '''<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
</model>
'''

MODEL_SDF = '''<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <pose>0 0 0.01 0 0 0</pose>
    <link name="base">
      <visual name="visual">
        <geometry><plane><normal>0 0 1</normal><size>{size} {size}</size></plane></geometry>
        <material>
          <diffuse>1 1 1 1</diffuse>
          <pbr><metal><albedo_map>model://{name}/{texture}</albedo_map></metal></pbr>
        </material>
      </visual>
    </link>
  </model>
</sdf>
'''

aruco_dict = aruco.getPredefinedDictionary(DICTIONARY)

for marker_id in range(1, MARKER_COUNT + 1):
    name = f'corridor_marker_{marker_id}'
    texture = f'marker_{marker_id}.png'
    model_dir = os.path.join(MODELS_DIR, name)
    os.makedirs(model_dir, exist_ok=True)

    image = aruco.generateImageMarker(aruco_dict, marker_id, MARKER_PX)
    image = cv2.copyMakeBorder(image, BORDER_PX, BORDER_PX, BORDER_PX,
                               BORDER_PX, cv2.BORDER_CONSTANT,
                               value=[255, 255, 255])
    cv2.imwrite(os.path.join(model_dir, texture), image)

    with open(os.path.join(model_dir, 'model.sdf'), 'w') as handle:
        handle.write(MODEL_SDF.format(name=name, texture=texture,
                                      size=MARKER_SIZE_M))
    with open(os.path.join(model_dir, 'model.config'), 'w') as handle:
        handle.write(MODEL_CONFIG.format(name=name))

    # Remove the shared-name texture left by earlier versions, so nothing can
    # load it by accident.
    stale = os.path.join(model_dir, 'marker.png')
    if os.path.exists(stale):
        os.remove(stale)

    print(f'{name}: ArUco id {marker_id}, texture {texture}')

print(f'\n{MARKER_COUNT} marker models written to {MODELS_DIR}')
print('Each uses a unique texture filename, so the Gazebo texture cache '
      'cannot collapse them into one.')
