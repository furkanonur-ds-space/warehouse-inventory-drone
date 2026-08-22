import os, qrcode, json

GZ_MODELS = os.path.expanduser('~/PX4-Autopilot/Tools/simulation/gz/models')

BOX_SIZE = 0.3
QR_PRINT_SIZE = 0.14   # matches the label size used on the reference warehouse
PLATE_THICKNESS = 0.04
LEVELS_Z = [0.35, 1.0, 1.65]   # shelf plate centre heights
ISLAND_DEPTH = 1.6
islands_x = [-4.0, 0.0, 4.0]
box_y_positions = [-4.0, 0.0, 4.0]

inventory = []
box_counter = 0

for island_i, cx in enumerate(islands_x):
    for level_i, plate_z in enumerate(LEVELS_Z):
        # The box sits on top of the plate
        box_center_z = plate_z + PLATE_THICKNESS/2 + BOX_SIZE/2
        for y in box_y_positions:
            for face in ['left', 'right']:
                box_counter += 1
                box_id = f"BOX-I{island_i+1}-L{level_i+1}-{face.upper()}-{box_counter:03d}"

                model_name = f"invbox_{box_counter:03d}"
                model_dir = os.path.join(GZ_MODELS, model_name)
                os.makedirs(model_dir, exist_ok=True)

                qr = qrcode.QRCode(box_size=10, border=1)
                qr.add_data(box_id)
                qr.make(fit=True)
                img = qr.make_image(fill_color='black', back_color='white').convert('RGB').resize((300, 300))
                # Unique filename is essential. Gazebo caches textures by filename,
                # so identical names collide and every box shows the same code.
                qr_filename = f'qr_{box_counter:03d}.png'
                qr_path = os.path.join(model_dir, qr_filename)
                img.save(qr_path)

                config = f'''<?xml version="1.0"?>
<model>
  <name>{model_name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
</model>'''
                with open(os.path.join(model_dir, 'model.config'), 'w') as f:
                    f.write(config)

                # The QR plane is mounted on the outward-facing side of the box
                # left  -> local -X face, normal points -X (pitch -90)
                # right -> local +X face, normal points +X (pitch +90)
                if face == 'left':
                    qr_local_x = -BOX_SIZE/2 - 0.002
                    pitch = -1.5708
                else:
                    qr_local_x = BOX_SIZE/2 + 0.002
                    pitch = 1.5708

                sdf = f'''<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <model name="{model_name}">
    <static>true</static>
    <link name="link">
      <visual name="box_visual">
        <geometry><box><size>{BOX_SIZE} {BOX_SIZE} {BOX_SIZE}</size></box></geometry>
        <material>
          <ambient>0.65 0.5 0.3 1</ambient>
          <diffuse>0.7 0.55 0.35 1</diffuse>
        </material>
      </visual>
      <collision name="box_collision">
        <geometry><box><size>{BOX_SIZE} {BOX_SIZE} {BOX_SIZE}</size></box></geometry>
      </collision>
      <visual name="qr_visual">
        <pose>{qr_local_x} 0 0 0 {pitch} 0</pose>
        <geometry><plane><normal>0 0 1</normal><size>{QR_PRINT_SIZE} {QR_PRINT_SIZE}</size></plane></geometry>
        <material>
          <diffuse>1 1 1 1</diffuse>
          <pbr><metal><albedo_map>model://{model_name}/{qr_filename}</albedo_map></metal></pbr>
        </material>
      </visual>
    </link>
  </model>
</sdf>'''
                with open(os.path.join(model_dir, 'model.sdf'), 'w') as f:
                    f.write(sdf)

                # Sit the box on the plate, close to the outward-facing edge
                box_edge_offset = ISLAND_DEPTH/2 - BOX_SIZE/2 - 0.02
                x_offset = -box_edge_offset if face == 'left' else box_edge_offset
                world_x = cx + x_offset

                inventory.append({
                    "id": box_id, "model": model_name,
                    "x": world_x, "y": y, "z": box_center_z,
                    "face": face, "island": island_i+1, "level": level_i+1
                })

print(f"Generated {box_counter} box models (body + QR combined)")

with open(os.path.expanduser('~/autonomous_landing/inventory_ground_truth.json'), 'w') as f:
    json.dump(inventory, f, indent=2, ensure_ascii=False)

print("Ground-truth inventory written")
