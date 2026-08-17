import os

GZ_MODELS = os.path.expanduser('~/PX4-Autopilot/Tools/simulation/gz/models')
model_name = "x500_scanner"
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

# Kamera sensor bloğu - tekrar kullanmak icin fonksiyon
def camera_block(link_name, joint_name, y_offset, yaw):
    return f'''
    <joint name="{joint_name}" type="fixed">
      <parent>base_link</parent>
      <child>{link_name}</child>
      <pose relative_to="base_link">0.05 {y_offset} -0.02 0 0 {yaw}</pose>
    </joint>
    <link name="{link_name}">
      <pose relative_to="{joint_name}">0 0 0 0 0 0</pose>
      <inertial>
        <mass>0.01</mass>
        <inertia>
          <ixx>0.00001</ixx><iyy>0.00001</iyy><izz>0.00001</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <sensor name="camera" type="camera">
        <gz_frame_id>{link_name}</gz_frame_id>
        <always_on>1</always_on>
        <update_rate>30</update_rate>
        <camera name="camera">
          <horizontal_fov>1.0472</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
            <format>R8G8B8</format>
          </image>
          <clip><near>0.1</near><far>100</far></clip>
        </camera>
      </sensor>
    </link>'''

left_cam = camera_block("camera_left_link", "camera_left_joint", 0.12, 1.5708)
right_cam = camera_block("camera_right_link", "camera_right_joint", -0.12, -1.5708)

sdf = f'''<?xml version="1.0" encoding="UTF-8"?>
<sdf version='1.9'>
  <model name='{model_name}'>
    <self_collide>false</self_collide>
    <include merge='true'>
      <uri>x500_flow</uri>
    </include>
{left_cam}
{right_cam}
  </model>
</sdf>'''
with open(os.path.join(model_dir, 'model.sdf'), 'w') as f:
    f.write(sdf)

print(f"{model_name} modeli hazir!")
print("- x500_flow tabanli (optical flow + LW20 mesafe sensoru)")
print("- SOL kamera eklendi (yaw=+90, sola bakiyor)")
print("- SAG kamera eklendi (yaw=-90, saga bakiyor)")
