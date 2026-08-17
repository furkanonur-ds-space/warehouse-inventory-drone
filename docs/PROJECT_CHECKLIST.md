# Warehouse Inventory Drone — Project Checklist

Status legend: `[x]` done · `[~]` partially done · `[ ]` not started

---

## Phase 1 — Environment Setup

- [x] Install WSL2 + Ubuntu 22.04
- [x] Install PX4 Autopilot (SITL build)
- [x] Install Gazebo Harmonic
- [x] Install MAVSDK-Python, OpenCV, NumPy
- [x] Verify basic SITL flight (`make px4_sitl gz_x500`)
- [x] Diagnose WSL2 GPU/performance limits
- [~] Optimize simulation performance (partially improved, still a constraint)

## Phase 2 — Earlier Tasks (context)

- [x] Task 1 — Precision landing on ArUco marker
- [x] Task 2 — Patrol flight with red-box detection
- [x] Task 3 — Two-drone swarm flight (async control)
- [x] ArUco marker comparison study (IDs, sizes, distances, angles, lighting)
- [x] Fiducial marker comparison report (ArUco / AprilTag / QR / barcode)
- [x] Roboflow research note

## Phase 3 — Warehouse Environment

- [x] Design warehouse layout (3 double-sided islands, 4 aisles)
- [x] Build shelf island models (`build_islands.py`)
- [x] Generate 54 boxes with unique QR codes (`build_boxes.py`)
- [x] Fix texture cache collision (unique filename per QR)
- [x] Place 8 ArUco floor markers at aisle ends
- [x] Generate marker position map (`marker_map.json`)
- [x] Assemble Gazebo world (`build_v2_world.py`)
- [x] Generate ground-truth inventory for validation

## Phase 4 — Vehicle Configuration

- [x] Build custom model with left/right cameras (`build_scanner_drone.py`)
- [x] Register custom PX4 airframe (`4022_gz_x500_scanner`)
- [x] Disable GPS (`SYS_HAS_GPS`, `SIM_GPS_USED`, `EKF2_GPS_CTRL`)
- [x] Verify optical flow and range sensor topics publish
- [x] Diagnose and fix EKF2 initialization failure
- [ ] Add downward-facing camera (required for ArUco correction)

## Phase 5 — Navigation

- [x] Verify GPS-free takeoff and hover
- [x] Measure position-hold drift (~0.05 m over 10 s)
- [x] Implement waypoint navigation (`waypoint_nav.py`)
- [x] Design boustrophedon route (4 aisles x 3 levels)
- [x] Replace stepwise motion with carrot-following (smooth cruise)
- [x] Correct aisle 4 geometry (too far from shelf)
- [x] Reach 24/24 waypoints with no collision

### Rejected approaches (documented, not deleted)
- [x] Lidar wall-following — abandoned, unreliable at aisle corners
- [x] Reinforcement learning (PPO, PyBullet) — abandoned, sparse-reward failure
- [x] Curriculum learning for RL — attempted, did not resolve

## Phase 6 — Perception

- [x] Subscribe to left/right camera streams
- [x] Diagnose QR decode failure (Gazebo renders white as grey)
- [x] Implement detect-crop-threshold decoding pipeline
- [x] Reach 52/54 codes decoded (96%)
- [ ] Recover remaining 2 missed codes
- [ ] Implement ArUco detection for drift correction
- [ ] Implement EKF position reset on marker sighting

## Phase 7 — Output

- [x] Export inventory as JSON (ID, estimated position, camera, timestamp)
- [x] Compare scanned result against ground truth
- [ ] Improve position estimate accuracy (currently fixed 0.8 m lateral offset)
- [ ] 3D visualization of the inventory map
- [ ] Interface for a warehouse management system

## Phase 8 — Documentation

- [x] Status report (interim)
- [x] Academic project report
- [x] Code delivery package with README
- [x] This checklist
- [ ] Line-by-line code walkthrough (in progress)

---

## Phase 9 — Next: ModalAI Hardware Platform

**What it is.** ModalAI is a US hardware manufacturer producing integrated
autopilot and perception computers for drones. Their VOXL 2 board runs flight
control and onboard computer vision on the same module. Gather AI, a commercial
warehouse inventory company solving exactly the problem this project addresses,
uses ModalAI Starling 2 Logis drones built on VOXL 2.

**Why it matters here.** The next step is to move the software developed in
simulation onto real ModalAI hardware. Published figures for the commercial
system give us benchmarks to measure against.

| Metric | Gather AI / Starling 2 Logis | This project (simulation) |
|---|---|---|
| Inventory accuracy | 99.9% | 96% (52/54) |
| Scan rate | up to 1,500 pallets/hour | not yet measured |
| Minimum aisle width | 1.35 m | 3.0 m (design choice) |
| Dynamic obstacle avoidance | yes | not implemented |
| Depth data | yes | not used |

- [ ] Study VOXL 2 architecture and developer documentation
- [ ] Check PX4 compatibility (VOXL 2 runs PX4, so the airframe config should port)
- [ ] Identify which parts of the code run on the flight controller vs the companion computer
- [ ] Evaluate whether the QR pipeline fits the VOXL 2 compute budget
- [ ] Review Starling 2 Logis camera configuration against our two-camera design
- [ ] Investigate VIO (visual-inertial odometry) as an alternative to optical flow
- [ ] Benchmark our scan rate against the 1,500 pallets/hour figure

## Phase 10 — Next: Hardware-in-the-Loop (HIL)

**What it means.** Run the real firmware on a real flight controller board
(e.g. Pixhawk) connected over USB. The board receives simulated sensor data
instead of real sensors, and its outputs drive the simulated vehicle. Motors do
not spin. This checks that the code meets real-time constraints on real
hardware, which SITL cannot verify.

- [ ] Obtain flight controller hardware (Pixhawk or equivalent)
- [ ] Flash PX4 firmware and enable HIL mode
- [ ] Configure Gazebo to feed sensor data to the board
- [ ] Verify EKF2 behaves identically to SITL
- [ ] Measure CPU load and loop timing on the board
- [ ] Confirm the QR pipeline runs within the frame budget on the companion computer

## Phase 11 — Next: Dynamic Obstacles

**What it means.** The current warehouse is entirely static, so a pre-planned
route is safe. A real warehouse contains people, forklifts and moving pallets
that cannot be planned around in advance. The vehicle needs a reactive safety
layer that overrides the planned route when something unexpected appears.

- [ ] Add moving obstacles to the Gazebo world (walking person, forklift)
- [ ] Add forward-facing depth sensor or lidar for obstacle detection
- [ ] Implement a safety layer that overrides the planned route
- [ ] Define recovery behaviour (hold, reroute, or resume)
- [ ] Test: does the vehicle still complete the scan after an interruption?
- [ ] Test: what happens when an aisle is fully blocked?

---

## Phase 12 — Longer Term

- [ ] Multi-drone concurrent scanning (split the warehouse into sectors)
- [ ] Compare QR against AprilTag for range and robustness
- [ ] Read floor plan from configuration rather than hard-coded values
- [ ] Battery-aware mission planning (return to charge, resume)
- [ ] Real flight test
