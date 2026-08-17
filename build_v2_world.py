import os, json

WORLDS = os.path.expanduser('~/PX4-Autopilot/Tools/simulation/gz/worlds')
INV_PATH = os.path.expanduser('~/autonomous_landing/inventory_ground_truth.json')

with open(INV_PATH) as f:
    inventory = json.load(f)

includes = []
counter = {}

def add_include(model_type, x, y, z, roll=0, pitch=0, yaw=0):
    counter[model_type] = counter.get(model_type, 0) + 1
    unique_name = f"{model_type}_{counter[model_type]}"
    includes.append(f'''    <include>
      <uri>model://{model_type}</uri>
      <name>{unique_name}</name>
      <pose>{x} {y} {z} {roll} {pitch} {yaw}</pose>
    </include>''')

# --- OUTER WALLS (20x20 m enclosure) ---
add_include('warehouse_wall', 0, 10, 2, 0, 0, 0)
add_include('warehouse_wall', 0, -10, 2, 0, 0, 0)
add_include('warehouse_wall', 10, 0, 2, 0, 0, 1.5708)
add_include('warehouse_wall', -10, 0, 2, 0, 0, 1.5708)

# --- THREE SHELF ISLANDS ---
islands_x = [-6.2, -1.6, 3.0]
for i, cx in enumerate(islands_x):
    add_include(f'island_{i+1}', cx, 0, 0, 0, 0, 0)

# --- 54 BOXES, positions read from ground truth ---
for item in inventory:
    add_include(item['model'], item['x'], item['y'], item['z'], 0, 0, 0)

# --- ARUCO FLOOR MARKERS, one at each end of each aisle ---
# Eight positions, eight UNIQUE ids, so a sighting is unambiguous
corridor_x_centers = [-8.5, -3.9, 0.7, 5.3]
marker_id = 1
marker_map = {}   # {aruco_id: {"x":..., "y":...}} used for drift correction
for cxc in corridor_x_centers:
    for y_end in [-8.0, 8.0]:
        marker_model = f"corridor_marker_{marker_id}"
        add_include(marker_model, cxc, y_end, 0, 0, 0, 0)
        marker_map[str(marker_id)] = {"x": cxc, "y": y_end}
        marker_id += 1

# Write the marker map for the flight code to consult
import json as _json
_map_path = os.path.expanduser('~/autonomous_landing/marker_map.json')
with open(_map_path, 'w') as _f:
    _json.dump(marker_map, _f, indent=2)
print(f"Marker map written to {_map_path}")

includes_str = '\n'.join(includes)

world_content = f'''<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <world name="warehouse_v2">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <grid>false</grid>
      <ambient>0.5 0.5 0.5 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>1 1</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>25 25</size></plane></geometry>
          <material>
            <ambient>0.6 0.6 0.6 1</ambient>
            <diffuse>0.65 0.65 0.65 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>0.3 0.3 -0.9</direction>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <attenuation><range>2000</range><linear>0</linear><constant>1</constant><quadratic>0</quadratic></attenuation>
    </light>

    <light name="indoor_light_1" type="point">
      <pose>0 0 3.5 0 0 0</pose>
      <cast_shadows>false</cast_shadows>
      <intensity>1.5</intensity>
      <diffuse>0.9 0.9 0.85 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation><range>15</range><linear>0.1</linear><constant>0.3</constant><quadratic>0.01</quadratic></attenuation>
    </light>

    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg>8.546163739800146</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>

{includes_str}

  </world>
</sdf>
'''

path = os.path.join(WORLDS, 'warehouse_v2.sdf')
with open(path, 'w') as f:
    f.write(world_content)

print("warehouse_v2.sdf generated")
print("3 islands, 54 boxes, 8 aisle-end markers")
