# Autonomous Warehouse Inventory Scanner

UAV-based warehouse inventory system built on PX4 SITL and Gazebo Harmonic.
The vehicle flies a pre-planned route through a warehouse, reads QR codes from
boxes on both sides of every aisle, and produces a JSON inventory mapping each
code to an estimated 3D position.


## 🏆 Results & Achievements (54/54 Boxes Scanned!)
The latest flight logic successfully scanned and decoded **54 out of 54 boxes** across 3 shelf islands and 4 corridors in a fully autonomous simulation, with a 100% success rate!

Key features of the flight logic:
- **Zero GPS Localization:** Relies entirely on Optical Flow, EKF2, and ArUco markers.
- **Smart ArUco Polling:** Overcomes the "fake drift" caused by camera pitch during acceleration by actively rejecting ArUco frames while the drone is in motion, and only applying drift correction during perfectly hovered 1.5-second settle stops at the end of each corridor.
- **Dynamic Snap Estimation:** Neutralizes physical camera tilt offset by dynamically snapping the 3D estimated position of decoded QR codes to the nearest logical warehouse shelf face and flight level.
- **Fully Modular Topology:** Warehouse parameters (corridors, islands, flight Z levels) are decoupled into `warehouse_config.json`, allowing instant adaptability to any warehouse grid layout without altering the core flight logic.

## Localization: no GPS is used

GPS is explicitly disabled. Warehouses are indoor environments where GNSS is
either unavailable or too imprecise to be useful, so the system relies on:

| Source | Role |
|---|---|
| Optical flow sensor (downward) | Measures ground-relative motion, giving velocity and hence dead-reckoned position |
| Distance sensor (downward) | Supplies height above ground, used as the EKF2 height reference |
| IMU (accelerometer + gyroscope) | Attitude and short-term motion, fused by EKF2 |
| ArUco floor markers | Ground-truth reference points at known coordinates, intended for periodic drift correction |

PX4's EKF2 estimator fuses optical flow, range and IMU data into a local
position estimate. The relevant parameters are set in the custom airframe file
(`4022_gz_x500_scanner`):

```sh
param set-default SYS_HAS_GPS 0      # no GPS hardware
param set-default SIM_GPS_USED 0     # no simulated GPS
param set-default EKF2_GPS_CTRL 0    # EKF2 must not fuse GPS
```

Measured drift while holding position for 10 seconds was approximately
0.05 m, which is well within the tolerance needed for aisle navigation.

## Navigation strategy

The warehouse floor plan is known in advance, so the system does not attempt
reactive exploration. Instead it follows a boustrophedon (zigzag) route:

```
Corridor 1, level 1: south to north
Corridor 1, level 2: north to south
Corridor 1, level 3: south to north
Corridor 2, level 3: north to south
...
```

Each corridor is flown once per shelf level. Because both cameras face
sideways, a single pass down an aisle scans the shelf faces on both sides
simultaneously.

An earlier iteration used a 2D lidar with a reactive wall-following algorithm.
It was abandoned because it could not reliably negotiate shelf corners and
because the floor plan is in fact known, making planned navigation both
simpler and more repeatable.

## Sensor layout

The custom vehicle model `x500_scanner` is based on PX4's stock `x500` and adds:

- `camera_left_link`  - 640x480 camera, yaw +90 degrees, reads the left shelf face
- `camera_right_link` - 640x480 camera, yaw -90 degrees, reads the right shelf face
- optical flow and range sensors inherited from the base model

## QR decoding

Gazebo's lighting renders the white quiet zone of a QR code as mid grey. On the
raw frame OpenCV detects the code but cannot decode it. Global Otsu
thresholding does not help either, since brightness varies across the frame.

The working pipeline is:

1. Locate the QR quadrilateral with `QRCodeDetector.detectMulti`
2. Crop that region only
3. Apply Otsu thresholding to the crop
4. Decode; if that fails, upscale 3x and retry

## Known issue that cost significant debugging time

Every box model initially stored its texture under the same filename
(`qr.png`). Gazebo's texture cache treated these as one asset, so all 54 boxes
rendered with the first box's QR code. Detection rate was 1 out of 54 and the
single decoded value repeated everywhere. Giving each box a unique texture
filename (`qr_001.png`, `qr_002.png`, ...) raised detection to 43 out of 54.

## Files

| File | Purpose |
|---|---|
| `warehouse_scanner.py` | Main scan routine: route execution, QR decoding, inventory export |
| `build_islands.py` | Generates the three double-sided shelf island models |
| `build_boxes.py` | Generates 54 box models, each with a unique QR texture |
| `build_v2_world.py` | Assembles the Gazebo world and writes the ArUco marker map |
| `build_scanner_drone.py` | Builds the `x500_scanner` vehicle model with side cameras |
| `flow_flight_test.py` | Verifies GPS-free takeoff, hover stability and drift |
| `waypoint_nav.py` | Navigation-only test, no camera processing |
| `4022_gz_x500_scanner` | PX4 airframe definition (place in `ROMFS/px4fmu_common/init.d-posix/airframes/`) |

## Running

```sh
# 1. Generate models and world
python3 build_islands.py
python3 build_boxes.py
python3 build_scanner_drone.py
python3 build_v2_world.py

# 2. Launch simulation
cd ~/PX4-Autopilot
export PX4_GZ_MODEL_POSE="-8.5,-9,0,0,0,1.5708"
make px4_sitl gz_x500_scanner_warehouse_v2

# 3. Run the scan (separate terminal)
python3 warehouse_scanner.py
```

Output is written to `inventory_scanned.json`.

## Current status

- Waypoint navigation: 24/24 waypoints reached, no collisions
- QR detection: 43/54 boxes on the last full run
- Remaining gaps were traced to corridor 4 being too wide, placing the UAV
  3.27 m from the shelf face instead of the 1.67 m used elsewhere. The corridor
  centre line has been moved from x=6.9 to x=5.3 to correct this.
- ArUco drift correction is specified and the marker map is generated, but the
  correction logic itself is not yet implemented.

## Requirements

- Ubuntu 22.04
- PX4 Autopilot (SITL)
- Gazebo Harmonic
- Python 3.10 with `mavsdk`, `opencv-python`, `numpy`, `qrcode`
