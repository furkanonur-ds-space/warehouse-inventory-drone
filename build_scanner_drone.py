"""
Build the x500_scanner vehicle model, mirroring the C27 sensor configuration.

C27 on the real vehicle carries:

    1 x IMX412   high resolution colour, front facing, used for scanning
    1 x TOF      depth, front facing, used for obstacle detection
    3 x AR0144   mono global shutter tracking cameras: front, rear, down

The tracking cameras are for localization, not for reading labels. Only the
IMX412 has the resolution to decode a 14 cm QR code at aisle range.

This changes the scanning strategy. With side-facing cameras a single pass down
an aisle covered both shelf faces at once. With a single front-facing camera the
vehicle must face the shelf it is scanning, so each shelf face needs its own
pass and the mission takes roughly twice as long.

The base model is x500 rather than x500_flow. x500_flow is broken in this PX4
version: EKF2 reports attitude 0 and never produces a position estimate, so the
vehicle cannot arm. Confirmed by running the stock x500_flow alone and seeing
the same failure. The plain x500 already provides optical flow and a range
sensor.
"""
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


def camera_block(link_name, joint_name, x_off, y_off, z_off,
                 roll, pitch, yaw, fov, width, height, update_rate=30):
    """
    One fixed-mounted camera.

    The joint is fixed because the cameras do not move relative to the
    airframe. The inertial values are deliberately tiny so the added links do
    not measurably change the flight dynamics.
    """
    return f'''
    <joint name="{joint_name}" type="fixed">
      <parent>base_link</parent>
      <child>{link_name}</child>
      <pose relative_to="base_link">{x_off} {y_off} {z_off} {roll} {pitch} {yaw}</pose>
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
        <update_rate>{update_rate}</update_rate>
        <camera name="camera">
          <horizontal_fov>{fov}</horizontal_fov>
          <image>
            <width>{width}</width>
            <height>{height}</height>
            <format>R8G8B8</format>
          </image>
          <clip><near>0.05</near><far>100</far></clip>
        </camera>
      </sensor>
    </link>'''


def range_block(link_name, joint_name, x_off, y_off, z_off,
                roll, pitch, yaw, max_range=10.0):
    """
    A single-beam range sensor standing in for the PMD TOF module.

    A full depth camera is not simulated here. For obstacle detection the
    relevant quantity is the distance to whatever is directly ahead, which a
    one-beam ray gives at a fraction of the cost.
    """
    return f'''
    <joint name="{joint_name}" type="fixed">
      <parent>base_link</parent>
      <child>{link_name}</child>
      <pose relative_to="base_link">{x_off} {y_off} {z_off} {roll} {pitch} {yaw}</pose>
    </joint>
    <link name="{link_name}">
      <pose relative_to="{joint_name}">0 0 0 0 0 0</pose>
      <inertial>
        <mass>0.005</mass>
        <inertia>
          <ixx>0.000005</ixx><iyy>0.000005</iyy><izz>0.000005</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <sensor name="tof" type="gpu_lidar">
        <gz_frame_id>{link_name}</gz_frame_id>
        <always_on>1</always_on>
        <update_rate>20</update_rate>
        <visualize>false</visualize>
        <ray>
          <scan>
            <horizontal>
              <samples>1</samples><resolution>1</resolution>
              <min_angle>0</min_angle><max_angle>0</max_angle>
            </horizontal>
            <vertical>
              <samples>1</samples><resolution>1</resolution>
              <min_angle>0</min_angle><max_angle>0</max_angle>
            </vertical>
          </scan>
          <range>
            <min>0.1</min><max>{max_range}</max><resolution>0.01</resolution>
          </range>
        </ray>
      </sensor>
    </link>'''


# --- IMX412, front facing, scanning ------------------------------------
#
# Resolution follows a readability budget rather than an arbitrary choice.
# A QR code needs roughly 3 pixels per module to decode. For a 14 cm code at
# the 1.37 m aisle standoff:
#
#     640x480,  60 deg -> 1.95 px/module, below threshold
#     1280x720, 60 deg -> 3.91 px/module, 30 percent margin
#
# The real IMX412 is 4056x3040, but its processing streams are downscaled to
# 1024x768, so 1280x720 is representative of what the pipeline actually sees.
hires_front = camera_block(
    "camera_hires_link", "camera_hires_joint",
    0.10, 0.0, 0.0, 0, 0, 0,
    fov=1.0472, width=1280, height=720)

# --- AR0144 tracking cameras -------------------------------------------
#
# 1280x800 mono on the real vehicle. Used for visual odometry, not for reading
# labels: at 1.37 m a 14 cm code resolves to about 2.6 px/module through these,
# which is below the decode threshold.
#
# Simulated at 640x480 because they feed motion estimation rather than
# perception, and the resolution saves render time that the hires camera needs.
tracking_front = camera_block(
    "camera_track_front_link", "camera_track_front_joint",
    0.08, 0.0, -0.03, 0, 0, 0,
    fov=1.5708, width=640, height=480)

tracking_rear = camera_block(
    "camera_track_rear_link", "camera_track_rear_joint",
    -0.08, 0.0, -0.03, 0, 0, 3.14159,
    fov=1.5708, width=640, height=480)

# The downward tracking camera doubles as the ArUco marker reader for drift
# correction. A 40 cm marker at 1.8 m altitude resolves to about 10 px/module
# through a 90 degree lens, well clear of the threshold.
tracking_down = camera_block(
    "camera_track_down_link", "camera_track_down_joint",
    0.0, 0.0, -0.05, 0, 1.5708, 0,
    fov=1.5708, width=640, height=480)

# --- TOF, front facing -------------------------------------------------
tof_front = range_block(
    "tof_link", "tof_joint",
    0.12, 0.0, 0.0, 0, 0, 0,
    max_range=10.0)

sdf = f'''<?xml version="1.0" encoding="UTF-8"?>
<sdf version='1.9'>
  <model name='{model_name}'>
    <self_collide>false</self_collide>
    <include merge='true'>
      <uri>x500</uri>
    </include>
{hires_front}
{tracking_front}
{tracking_rear}
{tracking_down}
{tof_front}
  </model>
</sdf>'''
with open(os.path.join(model_dir, 'model.sdf'), 'w') as f:
    f.write(sdf)

print(f"{model_name} model generated, C27 sensor configuration")
print("  camera_hires_link        1280x720  front, 60 deg   scanning")
print("  camera_track_front_link   640x480  front, 90 deg   odometry")
print("  camera_track_rear_link    640x480  rear,  90 deg   odometry")
print("  camera_track_down_link    640x480  down,  90 deg   odometry and ArUco")
print("  tof_link                  1 beam   front, 10 m     obstacles")
print()
print("  Note: scanning now requires the vehicle to face the shelf, so each")
print("  shelf face needs its own pass. Mission time roughly doubles.")
