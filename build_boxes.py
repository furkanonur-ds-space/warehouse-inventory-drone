import os, qrcode, json

GZ_MODELS = os.path.expanduser('~/PX4-Autopilot/Tools/simulation/gz/models')

BOX_SIZE = 0.3
PLATE_THICKNESS = 0.04
LEVELS_Z = [0.35, 1.0, 1.65]   # plaka MERKEZ yukseklikleri
ISLAND_DEPTH = 1.6
islands_x = [-6.2, -1.6, 3.0]
box_y_positions = [-4.0, 0.0, 4.0]

inventory = []
box_counter = 0

for island_i, cx in enumerate(islands_x):
    for level_i, plate_z in enumerate(LEVELS_Z):
        # Kutu, plakanin USTUNE oturur
        box_center_z = plate_z + PLATE_THICKNESS/2 + BOX_SIZE/2
        for y in box_y_positions:
            for face in ['sol', 'sag']:
                box_counter += 1
                box_id = f"KUTU-A{island_i+1}-K{level_i+1}-{face.upper()}-{box_counter:03d}"

                model_name = f"invbox_{box_counter:03d}"
                model_dir = os.path.join(GZ_MODELS, model_name)
                os.makedirs(model_dir, exist_ok=True)

                qr = qrcode.QRCode(box_size=10, border=1)
                qr.add_data(box_id)
                qr.make(fit=True)
                img = qr.make_image(fill_color='black', back_color='white').convert('RGB').resize((300, 300))
                # BENZERSIZ dosya adi sart! Ayni isimli texture'lari Gazebo
                # onbellekte cakistirip hepsine ayni goruntuyu uyguluyor.
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

                # QR, kutunun DISA BAKAN yuzune (yerel X ekseninde) monte edilir
                # sol -> yerel -X yuzu, normal -X'e baksin (pitch=-90)
                # sag -> yerel +X yuzu, normal +X'e baksin (pitch=+90)
                if face == 'sol':
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
        <geometry><plane><normal>0 0 1</normal><size>{BOX_SIZE*0.85} {BOX_SIZE*0.85}</size></plane></geometry>
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

                # Kutu, plakanin ust yuzeyinde, disa bakan kenara YAKIN dursun
                box_edge_offset = ISLAND_DEPTH/2 - BOX_SIZE/2 - 0.02
                x_offset = -box_edge_offset if face == 'sol' else box_edge_offset
                world_x = cx + x_offset

                inventory.append({
                    "id": box_id, "model": model_name,
                    "x": world_x, "y": y, "z": box_center_z,
                    "face": face, "island": island_i+1, "level": level_i+1
                })

print(f"Toplam {box_counter} kutu (govde+QR birlesik) modeli uretildi!")

with open(os.path.expanduser('~/autonomous_landing/inventory_ground_truth.json'), 'w') as f:
    json.dump(inventory, f, indent=2, ensure_ascii=False)

print("Envanter listesi guncellendi (yeni Z konumlariyla).")
