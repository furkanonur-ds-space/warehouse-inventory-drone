import os

GZ_MODELS = os.path.expanduser('~/PX4-Autopilot/Tools/simulation/gz/models')

ISLAND_DEPTH = 1.6    # X axis, the thin side
ISLAND_LENGTH = 12.0  # Y axis, the long side
ISLAND_HEIGHT = 2.0
LEG_SIZE = 0.08
PLATE_THICKNESS = 0.04
LEVELS_Z = [0.35, 1.0, 1.65]  # shelf plate heights

islands_x = [-4.0, 0.0, 4.0]   # 2.4 m aisles between and either side

# Legs: two edges in X (-depth/2, +depth/2), one every 3 m in Y
leg_y_positions = [-6, -2, 2, 6]
leg_x_offsets = [-ISLAND_DEPTH/2 + LEG_SIZE/2, ISLAND_DEPTH/2 - LEG_SIZE/2]

for idx, cx in enumerate(islands_x):
    model_name = f"island_{idx+1}"
    model_dir = os.path.join(GZ_MODELS, model_name)
    os.makedirs(model_dir, exist_ok=True)

    config = f'''<?xml version="1.0"?>
<model>
  <name>{model_name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
</model>'''
    with open(os.path.join(model_dir, 'model.config'), 'w') as f:
        f.write(config)

    links = []
    # Legs
    for ly in leg_y_positions:
        for lx in leg_x_offsets:
            links.append(f'''
      <visual name="leg_{ly}_{lx}_visual">
        <pose>{lx} {ly} {ISLAND_HEIGHT/2} 0 0 0</pose>
        <geometry><box><size>{LEG_SIZE} {LEG_SIZE} {ISLAND_HEIGHT}</size></box></geometry>
        <material><ambient>0.3 0.2 0.1 1</ambient><diffuse>0.35 0.22 0.12 1</diffuse></material>
      </visual>
      <collision name="leg_{ly}_{lx}_collision">
        <pose>{lx} {ly} {ISLAND_HEIGHT/2} 0 0 0</pose>
        <geometry><box><size>{LEG_SIZE} {LEG_SIZE} {ISLAND_HEIGHT}</size></box></geometry>
      </collision>''')

    # Horizontal shelf plates (3 levels)
    for z in LEVELS_Z:
        links.append(f'''
      <visual name="plate_{z}_visual">
        <pose>0 0 {z} 0 0 0</pose>
        <geometry><box><size>{ISLAND_DEPTH} {ISLAND_LENGTH} {PLATE_THICKNESS}</size></box></geometry>
        <material><ambient>0.55 0.4 0.25 1</ambient><diffuse>0.6 0.45 0.3 1</diffuse></material>
      </visual>
      <collision name="plate_{z}_collision">
        <pose>0 0 {z} 0 0 0</pose>
        <geometry><box><size>{ISLAND_DEPTH} {ISLAND_LENGTH} {PLATE_THICKNESS}</size></box></geometry>
      </collision>''')

    links_str = ''.join(links)

    sdf = f'''<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <model name="{model_name}">
    <static>true</static>
    <link name="body">
{links_str}
    </link>
  </model>
</sdf>'''
    with open(os.path.join(model_dir, 'model.sdf'), 'w') as f:
        f.write(sdf)

    print(f"Generated: {model_name} (legs and shelf plates)")

print("\n3 shelf island models generated")
