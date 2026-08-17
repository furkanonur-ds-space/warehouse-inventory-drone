import os

GZ_MODELS = os.path.expanduser('~/PX4-Autopilot/Tools/simulation/gz/models')

ISLAND_DEPTH = 1.6    # X ekseni (ince taraf)
ISLAND_LENGTH = 12.0  # Y ekseni (uzun taraf, kutularin dizildigi yon)
ISLAND_HEIGHT = 2.0
LEG_SIZE = 0.08
PLATE_THICKNESS = 0.04
LEVELS_Z = [0.35, 1.0, 1.65]  # plaka yukseklikleri

islands_x = [-6.2, -1.6, 3.0]

# Ayaklar: X'te 2 kenar (-depth/2, +depth/2), Y'de her 3m'de bir
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
    # Ayaklar
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

    # Yatay plakalar (3 seviye)
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

    print(f"Guncellendi: {model_name} (ayakli/raf gorunumu)")

print("\n3 ada RAF GORUNUMUNE cevrildi!")
