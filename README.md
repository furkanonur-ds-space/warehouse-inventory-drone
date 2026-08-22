# Autonomous Warehouse Inventory Scanner

UAV-based warehouse inventory system built on PX4 SITL and Gazebo Harmonic.
The vehicle flies a pre-planned route through a warehouse, reads QR codes from
boxes on both sides of every aisle, and produces a JSON inventory mapping each
code to an estimated 3D position.

Localization uses no GPS at any point. That claim is measured rather than
asserted; see "Evidence that no GPS is used" below.

## Results

Latest verified run, 2026-08-22:

| Metric | Value |
|---|---|
| Boxes decoded | 54 / 54 |
| Waypoints reached | 35 / 36 |
| Final drift offset | 0.097 m |
| Flight time | about 9 minutes of simulated time |

One waypoint reported a timeout instead of an arrival. A timeout means the leg
ran past its time budget, not that it was skipped: the setpoint keeps advancing
and the camera keeps being polled for the whole leg. Decoding is therefore
independent of whether arrival is confirmed within the budget, which is why the
run still decoded every box.

## Localization: no GPS

Warehouses are indoor environments where GNSS is either unavailable or too
imprecise to be useful. GPS is therefore disabled at three levels: the vehicle
declares no GPS hardware, the simulator publishes no usable fix, and EKF2 is
told not to fuse GPS even if one appears.

| Source | Role |
|---|---|
| Three tracking cameras | Visual odometry, simulated by Gazebo's OdometryPublisher and delivered to PX4 as external vision |
| IMU (accelerometer + gyroscope) | Attitude and short-term motion, fused by EKF2 |
| Barometer | Secondary height reference |
| ArUco floor markers | Ground-truth reference points at known coordinates, intended for periodic drift correction (see Known issues) |

The relevant parameters live in the custom airframe file
(`px4_config/4022_gz_x500_scanner`):

```sh
param set-default SYS_HAS_GPS 0      # no GPS hardware
param set-default SIM_GPS_USED 0     # no simulated GPS
param set-default EKF2_GPS_CTRL 0    # EKF2 must not fuse GPS
param set-default EKF2_EV_CTRL 15    # fuse external vision: position, velocity, yaw
param set-default EKF2_HGT_REF 3     # height reference is vision, not GPS
param set-default EKF2_OF_CTRL 0     # no optical flow on this airframe
```

`EKF2_EV_CTRL`, `EKF2_HGT_REF` and `EKF2_OF_CTRL` are not optional. Without
them EKF2 has no horizontal aiding source at all: `xy_valid` stays false, the
vehicle never gets a position, and the scan cannot start. This is a silent
failure, which is why `verify_setup.py` exists.

## Evidence that no GPS is used

Two independent measurements, not configuration claims.

**From the flight log.** EKF2 records which sources it fused. Over the 538
samples of a full scan (`estimator_status_flags`):

| Flag | Fraction of samples active |
|---|---|
| `cs_gnss_pos` | 0.000 |
| `cs_gps_hgt` | 0.000 |
| `cs_gnss_yaw` | 0.000 |
| `cs_ev_pos` | 0.993 |
| `cs_ev_vel` | 0.993 |
| `cs_ev_hgt` | 0.998 |
| `cs_opt_flow` | 0.000 |
| `xy_valid` | 0.999 |

GPS contributed to nothing. Position came from external vision throughout.

**From the running estimator.** Queried live, mid-flight:

```
cs_gnss_pos: False        xy_valid:   True
cs_ev_pos:   True         v_xy_valid: True
cs_ev_vel:   True
cs_ev_hgt:   True
cs_opt_flow: False
```

`vehicle_global_position` is never published, and `ref_lat` / `ref_lon` stay
`nan`, meaning EKF2 has no global origin to work from.

To reproduce either check, run `verify_setup.py` before a measurement run, or
inspect a `.ulg` from `~/PX4-Autopilot/build/px4_sitl_default/rootfs/log/`.

## Navigation strategy

The warehouse floor plan is known in advance, so the system does not attempt
reactive exploration. Instead it follows a boustrophedon route. Because the
scanning camera faces forward rather than sideways, the vehicle must turn to
face each shelf and fly sideways along it, so every shelf face needs its own
pass:

```
Face 1 (aisle -6, facing east), level 1: south to north
Face 1 (aisle -6, facing east), level 2: north to south
Face 1 (aisle -6, facing east), level 3: south to north
Face 2 (aisle -2, facing west), level 3: north to south
...
```

Both the along-aisle direction and the level order alternate, so the vehicle
never flies an empty leg and never drops back to the bottom shelf when it
starts a new face.

An earlier iteration used a 2D lidar with reactive wall following. It was
abandoned because it could not reliably negotiate shelf corners, and because
the floor plan is in fact known, making planned navigation both simpler and
more repeatable.

Warehouse geometry is decoupled into `warehouse_config.json` (corridors,
islands, flight levels), so a different grid layout needs no code changes.

## Sensor layout

The custom vehicle model `x500_scanner` is based on PX4's stock `x500` and
mirrors the C27 sensor configuration:

- `camera_hires_link` - 1280x720, front, 60 degrees, the scanning camera
- `camera_track_front_link` - 640x480, front, 90 degrees, odometry
- `camera_track_rear_link` - 640x480, rear, 90 degrees, odometry
- `camera_track_down_link` - 640x480, down, 90 degrees, odometry and ArUco
- `OdometryPublisher` plugin - stands in for the VOXL 2 computing VIO

## QR decoding

Gazebo's lighting renders the white quiet zone of a QR code as mid grey. On the
raw frame OpenCV detects the code but cannot decode it. Global Otsu
thresholding does not help either, since brightness varies across the frame.

The working pipeline is:

1. Locate the QR quadrilateral with `QRCodeDetector.detectMulti`
2. Crop that region only
3. Apply Otsu thresholding to the crop
4. Decode; if that fails, upscale 3x and retry

## Position estimation

A code's position is derived from where it appears in the frame, not from an
assumption that it lies straight ahead. A first version did assume straight
ahead and added a fixed standoff along the heading; measured against ground
truth that gave a median error of 5.1 m, with individual errors up to 11.7 m,
almost entirely along the aisle. A code seen at the edge of a 60 degree frame
is far off to the side, and treating it as central misplaces it by metres.

The pose is captured in the camera callback rather than in the main loop.
Decoding takes time while the vehicle keeps moving, so looking the pose up
later attributes each box to wherever the vehicle had reached by then.

Because the vehicle pitches to fly, the camera tilts and introduces a false
bearing and elevation. Since the shelf grid is known, each estimate is snapped
to the nearest shelf face and flight level, which removes that error.

## Known issues

**ArUco drift correction is not functional.** All eight marker models reference
their texture as `marker.png`, the same basename in every model directory.
Gazebo's texture cache treats these as one asset, so every marker in the world
renders with marker 1's texture. Measured directly: with the vehicle 0.02 m
from marker 7 and 0.06 m from marker 8, the downward camera read id 1 in both
cases, 0 of 15 detections correct.

The consequence is contained rather than harmful. Sightings at the wrong
location disagree with the estimate by more than the 2 m plausibility gate and
are rejected, so no bad correction is applied. Visual odometry drift is small
enough (0.097 m over a full scan) that the scan succeeds without the
correction. The fix is the one already applied to the box textures: give each
marker a unique filename.

This is the same failure that once cost significant debugging time on the
boxes. Every box model initially stored its texture as `qr.png`, so all 54
boxes rendered with the first box's QR code; detection was 1 out of 54 and the
single decoded value repeated everywhere. Unique filenames (`qr_001.png`,
`qr_002.png`, and so on) fixed it.

**Gazebo memory growth.** `gz sim` grows to tens of gigabytes of resident
memory over a full scan and can be killed by the system or become
unresponsive. Restart the simulator between runs.

## Files

| File | Purpose |
|---|---|
| `warehouse_scanner.py` | Main scan routine: route execution, QR decoding, inventory export |
| `verify_setup.py` | Checks the GPS-free claim against the running system before a measurement run |
| `build_islands.py` | Generates the three double-sided shelf island models |
| `build_boxes.py` | Generates 54 box models, each with a unique QR texture |
| `build_v2_world.py` | Assembles the Gazebo world and writes the ArUco marker map |
| `build_scanner_drone.py` | Builds the `x500_scanner` vehicle model with the C27 camera set |
| `generate_markers.py` | Generates the eight ArUco floor marker textures |
| `warehouse_config.json` | Warehouse geometry: corridors, islands, flight levels |
| `px4_config/4022_gz_x500_scanner` | PX4 airframe definition |

## Setup

The airframe file is compiled into PX4's ROMFS, so it must be copied into the
PX4 tree and PX4 rebuilt. Keeping it only in the PX4 tree is how a working
configuration was once lost; it lives here so it survives.

```sh
cp px4_config/4022_gz_x500_scanner \
   ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/
```

Confirm it is registered in that directory's `CMakeLists.txt`, then generate
the models and world:

```sh
python3 build_islands.py
python3 build_boxes.py
python3 build_scanner_drone.py
python3 generate_markers.py
python3 build_v2_world.py
```

## Running

```sh
# Terminal 1: simulator. The first run after changing the airframe rebuilds PX4.
cd ~/PX4-Autopilot
export HEADLESS=1
export PX4_GZ_MODEL_POSE="-6.0,-9,0,0,0,1.5708"
make px4_sitl gz_x500_scanner_warehouse_v2
```

```sh
# Terminal 2: verify the configuration, then scan.
cd ~/autonomous_landing
source venv/bin/activate
python3 verify_setup.py
python3 warehouse_scanner.py
```

The virtual environment matters: the system Python has an OpenCV built against
NumPy 1.x alongside NumPy 2.x, and `import cv2` fails there.

Output is written to `inventory_scanned.json`.

## Requirements

- Ubuntu 22.04
- PX4 Autopilot (SITL)
- Gazebo Harmonic
- Python 3.10 with `mavsdk`, `opencv-python`, `numpy`, `qrcode`
