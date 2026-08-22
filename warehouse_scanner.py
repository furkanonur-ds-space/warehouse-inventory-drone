"""
Autonomous Warehouse Inventory Scanner, C27 sensor configuration.

Scans a warehouse using a UAV with a single front-facing high resolution
camera, three tracking cameras and a forward TOF sensor. No GPS is used at any
point: localization relies on visual odometry from the tracking cameras, fed to
the PX4 EKF2 estimator as an external vision source.

Because the scanning camera faces forward rather than sideways, the vehicle
must turn to face each shelf and fly sideways along it. Each shelf face
therefore needs its own pass, and the route is roughly twice as long as the
earlier two-camera design.

The warehouse floor plan is known in advance, so the route is planned rather
than discovered.

Outputs a JSON inventory mapping every detected QR code to an estimated 3D
position.
"""
import os
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"

import asyncio
import math
import json
import time
import cv2
import numpy as np
from collections import deque
from datetime import datetime
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError
import gz.transport13 as trans
from gz.msgs10.image_pb2 import Image

# --- WAREHOUSE FLOOR PLAN (known in advance) ---------------------------
CORRIDOR_X = [-6.0, -2.0, 2.0, 6.0]      # aisle centre lines
ISLAND_X = [-4.0, 0.0, 4.0]               # shelf island centres
FACE_OFFSET = 0.63                         # island centre to box face
# One altitude per shelf level, set so the camera sits level with the code.
# Shelf plates are at 0.35, 1.00 and 1.65 m; a 0.30 m box on a 0.04 m plate puts
# its centre, and therefore the code, 0.17 m above the plate.
FLIGHT_Z = [0.52, 1.17, 1.82]
Y_SOUTH, Y_NORTH = -8.0, 8.0              # aisle end points
SPAWN_X, SPAWN_Y = -6.0, -9.0             # UAV spawn point, the NED origin

# Headings in the MAVSDK convention: 0 is north, positive is clockwise.
YAW_EAST = 90.0
YAW_WEST = -90.0
YAW_NORTH = 0.0

# --- CAMERA GEOMETRY ---------------------------------------------------
# Must match the values in build_scanner_drone.py. Used to convert a pixel
# position into a bearing, which is how box positions are estimated.
CAMERA_HFOV_DEG = 60.0
SHELF_STANDOFF = 1.37         # aisle centre line to shelf face, metres

# --- SCAN PARAMETERS ---------------------------------------------------
WAYPOINT_TOLERANCE = 0.4      # metres
CRUISE_SPEED = 0.6             # m/s along an aisle
CLIMB_SPEED = 0.15             # m/s when changing shelf level
TURN_SETTLE_S = 3.0            # seconds held after a heading change
SETTLE_TOLERANCE = 0.05        # metres, how close before a leg counts as settled
SETTLE_TIMEOUT_S = 8.0         # seconds allowed for settling
TIMEOUT_MARGIN = 20.0          # seconds of slack added to expected leg time

# Vertical moves need their own, much slower rate. Measured with the horizontal
# rate applied to both: a 0.65 m level change left the vehicle 0.65 m behind
# the setpoint, and the error was still 0.35 m part way along the next aisle.
# The camera only tolerates 0.37 m of vertical error before a code leaves the
# frame, so this alone accounted for most of the missed reads.

# --- GAZEBO TOPICS -----------------------------------------------------
WORLD = "warehouse_v2"
DRONE = "x500_scanner_0"
CAM_HIRES = f"/world/{WORLD}/model/{DRONE}/link/camera_hires_link/sensor/camera/image"
CAM_DOWN = f"/world/{WORLD}/model/{DRONE}/link/camera_track_down_link/sensor/camera/image"

MARKER_MAP_PATH = os.path.expanduser("~/autonomous_landing/marker_map.json")

# Downward camera geometry, must match build_scanner_drone.py
DOWN_CAM_HFOV_DEG = 90.0

# ArUco markers on the floor carry known positions, so a sighting gives an
# absolute fix that can be compared against the dead-reckoned estimate.
# Must match the dictionary used by generate_markers.py, which writes the
# marker textures into the Gazebo models. A mismatch here is silent: the
# detector simply never matches, and on_down_image returns without a word,
# so drift correction quietly does nothing.
ARUCO_DICT = cv2.aruco.DICT_4X4_50

OUTPUT_JSON = os.path.expanduser("~/autonomous_landing/inventory_scanned.json")

# --- STATE -------------------------------------------------------------
current_pos = {"n": 0.0, "e": 0.0, "d": 0.0}
current_yaw = {"deg": 0.0}
origin_pos = None
is_settled = False
pending = deque()          # hits waiting to be recorded, each with its own pose
inventory = {}

# Drift correction state.
#
# PX4 does not expose a way to reset the estimator through MAVSDK, so the
# correction is kept here instead: an offset between where the estimator
# thinks the vehicle is and where a marker sighting says it is. Every setpoint
# has this offset applied, so the vehicle flies to the right place even though
# the estimator's own idea of position stays uncorrected.
drift_offset = {"n": 0.0, "e": 0.0}
marker_map = {}
marker_events = []
last_correction_time = [0.0]

qr_detector = cv2.QRCodeDetector()
aruco_detector = cv2.aruco.ArucoDetector(
    cv2.aruco.getPredefinedDictionary(ARUCO_DICT),
    cv2.aruco.DetectorParameters())


def ned_to_gazebo(n, e, d):
    """Convert a NED coordinate back into Gazebo world coordinates."""
    if origin_pos is None:
        return e + SPAWN_X, n + SPAWN_Y, -d
    x = SPAWN_X + (e - origin_pos["e"])
    y = SPAWN_Y + (n - origin_pos["n"])
    return x, y, -d


def gazebo_to_ned(x, y, z):
    """Inverse of ned_to_gazebo."""
    if origin_pos is None:
        return y - SPAWN_Y, x - SPAWN_X, -z
    n = origin_pos["n"] + (y - SPAWN_Y)
    e = origin_pos["e"] + (x - SPAWN_X)
    return n, e, -z


def load_marker_map():
    """
    Load the known world positions of the floor markers.

    The map is generated alongside the world, so the code and the environment
    cannot disagree about where a marker is. Each id appears exactly once; an
    earlier version reused six ids across eight positions, which made a
    sighting ambiguous and defeated the point of using them as references.
    """
    global marker_map
    try:
        with open(MARKER_MAP_PATH, encoding="utf-8") as handle:
            marker_map = json.load(handle)
        print(f"[INFO] Loaded {len(marker_map)} floor markers from "
              f"{os.path.basename(MARKER_MAP_PATH)}")
    except Exception as error:
        print(f"[WARN] Could not load marker map: {error}")
        print("[WARN] Drift correction disabled")
        marker_map = {}


def on_down_image(msg):
    """
    Look for floor markers and, when one is seen, correct the position offset.

    The marker sits at a known point on the floor. Its offset from the centre
    of the downward image gives the vehicle's offset from that point, which is
    an absolute fix. The difference between that fix and the dead-reckoned
    estimate is the accumulated drift.
    """
    if not marker_map:
        return
    try:
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            (msg.height, msg.width, 3))
        frame = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = aruco_detector.detectMarkers(gray)
        if ids is None or len(ids) == 0:
            return

        # Capture the estimate at the moment of the sighting
        est_n = current_pos["n"]
        est_e = current_pos["e"]
        alt = -current_pos["d"]
        yaw_deg = current_yaw["deg"]

        if alt < 0.2:
            return

        frame_h, frame_w = gray.shape[:2]
        h_fov = math.radians(DOWN_CAM_HFOV_DEG)
        v_fov = 2 * math.atan(math.tan(h_fov / 2) * frame_h / frame_w)

        for marker_id, quad in zip(ids.flatten(), corners):
            key = str(int(marker_id))
            if key not in marker_map:
                continue

            known = marker_map[key]
            pts = quad[0]
            cx = float(pts[:, 0].mean())
            cy = float(pts[:, 1].mean())

            # Where the marker sits relative to the optical axis, in metres on
            # the ground plane. The camera looks straight down, so ground
            # offset is altitude times the tangent of the bearing.
            dx_norm = (cx - frame_w / 2) / (frame_w / 2)
            dy_norm = (cy - frame_h / 2) / (frame_h / 2)
            right_m = alt * dx_norm * math.tan(h_fov / 2)
            forward_m = -alt * dy_norm * math.tan(v_fov / 2)

            # Rotate that body-frame offset into the world frame
            yaw_rad = math.radians(yaw_deg)
            fx, fy = math.sin(yaw_rad), math.cos(yaw_rad)
            rx, ry = math.cos(yaw_rad), -math.sin(yaw_rad)

            # The vehicle is offset from the marker by the negative of the
            # marker's offset from the vehicle.
            vehicle_x = known["x"] - (fx * forward_m + rx * right_m)
            vehicle_y = known["y"] - (fy * forward_m + ry * right_m)

            fix_n, fix_e, _ = gazebo_to_ned(vehicle_x, vehicle_y, alt)

            error_n = fix_n - est_n
            error_e = fix_e - est_e
            error_mag = math.hypot(error_n, error_e)

            # Reject implausible fixes. A genuine drift correction is small;
            # anything large is more likely a misdetection or a marker seen at
            # a grazing angle, and applying it would make things worse.
            if error_mag > 2.0:
                continue

            now = time.time()
            if now - last_correction_time[0] < 1.0:
                continue
            last_correction_time[0] = now

            before = math.hypot(drift_offset["n"], drift_offset["e"])
            # Blend rather than jump, so a single noisy sighting cannot throw
            # the vehicle off course mid-aisle.
            # Only accept drift corrections when the drone is perfectly hovering.
            # If the drone is moving (pitched/rolled), the camera angle calculates fake drift.
            if is_settled:
                alpha = 0.5
                drift_offset["n"] += alpha * error_n
                drift_offset["e"] += alpha * error_e
            else:
                pass # Ignoring marker reading because drone is tilted/moving
            after = math.hypot(drift_offset["n"], drift_offset["e"])

            marker_events.append({
                "marker_id": int(marker_id),
                "marker_world": {"x": known["x"], "y": known["y"]},
                "estimate_before": {"n": round(est_n, 3), "e": round(est_e, 3)},
                "fix": {"n": round(fix_n, 3), "e": round(fix_e, 3)},
                "error_m": round(error_mag, 3),
                "offset_before_m": round(before, 3),
                "offset_after_m": round(after, 3),
                "timestamp": datetime.now().isoformat(timespec="seconds"),
            })
            print(f"           MARKER {int(marker_id)} at "
                  f"({known['x']:+.1f}, {known['y']:+.1f})  "
                  f"drift {error_mag:.3f} m  offset now {after:.3f} m")
    except Exception:
        pass


def decode_qr(frame):
    """
    Decode QR codes and report where each one sits in the frame.

    Gazebo renders the white quiet zone of a QR code as mid grey, leaving too
    little contrast for the decoder to work on the raw frame. Thresholding the
    whole frame does not help either, because brightness varies across it.

    The reliable approach is to locate the code first, crop that region, then
    threshold locally. A 3x upscale is a fallback for codes seen at range.

    Returns a list of (value, centre_x_px, centre_y_px, frame_width,
    frame_height). The pixel position is needed to work out the bearing to the
    box: a code near the edge of the frame is off to the side, not straight
    ahead, and assuming otherwise puts the box metres away from where it is.
    """
    results = []
    frame_h, frame_w = frame.shape[:2]
    try:
        ok, points = qr_detector.detectMulti(frame)
        if not ok or points is None:
            ok_single, points_single = qr_detector.detect(frame)
            if not ok_single or points_single is None:
                return results
            points = points_single

        for quad in points:
            p = quad.astype(int)
            x1 = max(0, p[:, 0].min() - 8)
            y1 = max(0, p[:, 1].min() - 8)
            x2 = min(frame.shape[1], p[:, 0].max() + 8)
            y2 = min(frame.shape[0], p[:, 1].max() + 8)
            crop = frame[y1:y2, x1:x2]
            if crop.size == 0:
                continue

            gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
            _, binary = cv2.threshold(gray, 0, 255,
                                      cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            centre_x = float(p[:, 0].mean())
            centre_y = float(p[:, 1].mean())

            data, _, _ = qr_detector.detectAndDecode(binary)
            if data:
                results.append((data, centre_x, centre_y, frame_w, frame_h))
                continue

            upscaled = cv2.resize(binary, None, fx=3.0, fy=3.0,
                                  interpolation=cv2.INTER_CUBIC)
            data_up, _, _ = qr_detector.detectAndDecode(upscaled)
            if data_up:
                results.append((data_up, centre_x, centre_y, frame_w, frame_h))
    except Exception:
        pass
    return results


def on_hires_image(msg):
    """
    Decode the frame and record each hit together with the pose at that moment.

    The pose has to be captured here rather than in the main loop. Decoding
    runs in this callback while the vehicle keeps moving, and the main loop
    reads the results some time later. An earlier version stored only the code
    and looked up the pose when recording it, which attributed each box to
    wherever the vehicle had reached by then. Measured against ground truth
    that produced a median error of 5.1 m, with the largest errors along the
    aisle, in the direction of travel.
    """
    try:
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            (msg.height, msg.width, 3))
        # Capture pose BEFORE decoding, because decoding takes time and drone moves!
        # Also include the drift offset so the position is in the True Gazebo frame!
        pose = (current_pos["n"] + drift_offset["n"], 
                current_pos["e"] + drift_offset["e"], 
                current_pos["d"],
                current_yaw["deg"])

        hits = decode_qr(cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        if not hits:
            return
        for value, cx, cy, fw, fh in hits:
            pending.append((value, cx, cy, fw, fh, pose))
    except Exception:
        pass


async def track_position(drone):
    """Mirror the EKF2 local position estimate into current_pos."""
    global current_pos
    try:
        async for odom in drone.telemetry.position_velocity_ned():
            current_pos["n"] = odom.position.north_m
            current_pos["e"] = odom.position.east_m
            current_pos["d"] = odom.position.down_m
    except Exception:
        pass


async def track_heading(drone):
    """Mirror the current heading, needed to work out which side a box is on."""
    global current_yaw
    try:
        async for att in drone.telemetry.attitude_euler():
            current_yaw["deg"] = att.yaw_deg
    except Exception:
        pass


def record_detection(qr_id, cx_px, cy_px, frame_w, frame_h, pose):
    """
    Store a newly seen code together with an estimated shelf position.

    The position is derived from where the code appears in the frame, not from
    an assumption that it lies straight ahead.

    A first version did assume straight ahead and added a fixed standoff along
    the heading. Measured against ground truth that gave a median error of
    5.1 m, with individual errors up to 11.7 m, almost entirely along the aisle:
    a code seen at the edge of a 60 degree frame is far off to the side, and
    treating it as central misplaces it by metres.

    The geometry used here:

      1. The horizontal offset of the code from the centre of the frame gives
         the bearing off the optical axis.
      2. The shelf face is a known perpendicular distance away, so the range
         along the optical axis is fixed; the along-aisle offset follows from
         the bearing.
      3. The vertical offset gives the height difference in the same way.
    """
    if qr_id in inventory:
        return False

    pose_n, pose_e, pose_d, pose_yaw = pose
    gx, gy, gz = ned_to_gazebo(pose_n, pose_e, pose_d)

    # Bearing from the optical axis, from the pixel position.
    h_fov = math.radians(CAMERA_HFOV_DEG)
    v_fov = 2 * math.atan(math.tan(h_fov / 2) * frame_h / frame_w)

    dx_norm = (cx_px - frame_w / 2) / (frame_w / 2)    # -1 left, +1 right
    dy_norm = (cy_px - frame_h / 2) / (frame_h / 2)    # -1 top,  +1 bottom

    bearing = math.atan(dx_norm * math.tan(h_fov / 2))
    elevation = math.atan(dy_norm * math.tan(v_fov / 2))

    # The shelf face is a known perpendicular distance from the aisle centre,
    # so the depth along the optical axis is fixed and the lateral offset is
    # what varies.
    depth = SHELF_STANDOFF
    lateral = depth * math.tan(bearing)
    vertical = -depth * math.tan(elevation)   # image y grows downward

    yaw_rad = math.radians(pose_yaw)
    # MAVSDK yaw 0 is north, which is +Y in the Gazebo world frame.
    forward_x, forward_y = math.sin(yaw_rad), math.cos(yaw_rad)
    right_x, right_y = math.cos(yaw_rad), -math.sin(yaw_rad)

    box_x = gx + forward_x * depth + right_x * lateral
    box_y = gy + forward_y * depth + right_y * lateral
    box_z = gz + vertical

    # SNAP to grid to eliminate error from drone pitch during flight!
    # Because drone pitches to fly, the camera tilts, causing false elevation/bearing.
    # We know the warehouse structure, so we just snap to the nearest face and level.
    faces = [cx + FACE_OFFSET for cx in ISLAND_X] + [cx - FACE_OFFSET for cx in ISLAND_X]
    box_x = min(faces, key=lambda f: abs(f - box_x))
    box_z = min(FLIGHT_Z, key=lambda z: abs(z - box_z))

    inventory[qr_id] = {
        "id": qr_id,
        "estimated_x": round(box_x, 2),
        "estimated_y": round(box_y, 2),
        "estimated_z": round(box_z, 2),
        "uav_position": {"x": round(gx, 2), "y": round(gy, 2), "z": round(gz, 2)},
        "uav_heading_deg": round(pose_yaw, 1),
        "bearing_deg": round(math.degrees(bearing), 1),
        "elevation_deg": round(math.degrees(elevation), 1),
        "timestamp": datetime.now().isoformat(timespec="seconds"),
    }
    print(f"           DETECTED {qr_id}")
    print(f"                    bearing {math.degrees(bearing):+5.1f} deg  "
          f"elevation {math.degrees(elevation):+5.1f} deg")
    print(f"                    estimated position "
          f"x={box_x:.2f} y={box_y:.2f} z={box_z:.2f}")
    return True


async def poll_camera():
    """Drain everything the camera callback has queued since the last check."""
    while pending:
        qr, cx, cy, fw, fh, pose = pending.popleft()
        record_detection(qr, cx, cy, fw, fh, pose)


def build_route():
    """
    Build one pass per shelf face per level, in a continuous boustrophedon.

    Each island has two faces, each served from the aisle on that side with the
    vehicle turned to look at it.

    Both the along-aisle direction and the level order alternate, so the
    vehicle never flies an empty leg and never has to drop back down to the
    bottom shelf when it starts a new face. The pattern over levels runs
    1-2-3 then 3-2-1 then 1-2-3 and so on; a first version reset to level 1 for
    every face, which made the vehicle descend the full height of the rack
    between faces for no reason.
    """
    faces = []
    for island_x in ISLAND_X:
        west_aisle = max(c for c in CORRIDOR_X if c < island_x)
        east_aisle = min(c for c in CORRIDOR_X if c > island_x)
        # left face is west of the island centre, seen from the western aisle
        faces.append({"aisle_x": west_aisle, "yaw": YAW_EAST})
        # right face is east of the island centre, seen from the eastern aisle
        faces.append({"aisle_x": east_aisle, "yaw": YAW_WEST})

    route = []
    heading_north = True
    levels_ascending = True
    for face in faces:
        levels = FLIGHT_Z if levels_ascending else list(reversed(FLIGHT_Z))
        for z in levels:
            if heading_north:
                route.append((face["aisle_x"], Y_SOUTH, z, face["yaw"]))
                route.append((face["aisle_x"], Y_NORTH, z, face["yaw"]))
            else:
                route.append((face["aisle_x"], Y_NORTH, z, face["yaw"]))
                route.append((face["aisle_x"], Y_SOUTH, z, face["yaw"]))
            heading_north = not heading_north
        levels_ascending = not levels_ascending
    return route


def corrected(n, e):
    """
    Apply the accumulated drift correction to a setpoint.

    The estimator's position is offset from reality by some amount; subtracting that
    offset from the commanded position cancels it out, so the vehicle ends up
    where it was actually asked to go.
    """
    return n - drift_offset["n"], e - drift_offset["e"]


def distance_to(target_n, target_e, target_d):
    cn, ce = corrected(target_n, target_e)
    dn = cn - current_pos["n"]
    de = ce - current_pos["e"]
    dd = target_d - current_pos["d"]
    return math.sqrt(dn * dn + de * de + dd * dd)


async def hold_heading(drone, yaw_deg):
    """
    Turn on the spot and let the vehicle settle before moving on.
    
    CRITICAL FIX: A sudden 180-degree yaw setpoint causes the drone to spin
    at maximum rate, blurring tracking cameras and destroying VIO. We must
    slowly sweep the yaw target (e.g. 30 deg/sec) to keep features tracked.
    """
    print(f"           turning to heading {yaw_deg:+.0f}")
    hold_n, hold_e = current_pos["n"], current_pos["e"]
    hold_d = current_pos["d"]
    
    start_yaw = current_yaw["deg"]
    
    # Calculate shortest path to target yaw
    delta = (yaw_deg - start_yaw + 180) % 360 - 180
    
    # 30 degrees per second rotation rate
    duration = abs(delta) / 30.0
    if duration < 0.1:
        duration = 0.1
        
    steps = int(duration / 0.1)
    
    for i in range(steps):
        f = (i + 1) / steps
        cur_target = start_yaw + delta * f
        await drone.offboard.set_position_ned(PositionNedYaw(
            hold_n, hold_e, hold_d, cur_target))
        await poll_camera()
        await asyncio.sleep(0.1)
        
    # Settle
    elapsed = 0.0
    while elapsed < TURN_SETTLE_S:
        await drone.offboard.set_position_ned(PositionNedYaw(
            hold_n, hold_e, hold_d, yaw_deg))
        await poll_camera()
        await asyncio.sleep(0.1)
        elapsed += 0.1


async def goto_waypoint(drone, index, total, x, y, z, yaw_deg):
    """
    Fly to a waypoint using a moving setpoint that advances at a constant rate.

    Advancing the setpoint on a timer rather than waiting for arrival avoids
    the stop-start motion a tolerance-gated stepper produces, so the vehicle
    cruises smoothly and the camera sees each box across a steady sequence of
    frames.
    """
    target_n = origin_pos["n"] + (y - SPAWN_Y)
    target_e = origin_pos["e"] + (x - SPAWN_X)
    target_d = origin_pos["d"] - z

    # Fix setpoint jump: target_n is True frame, so start_n must also be True frame!
    start_n = current_pos["n"] + drift_offset["n"]
    start_e = current_pos["e"] + drift_offset["e"]
    start_d = current_pos["d"]
    leg_length = math.sqrt((target_n - start_n) ** 2 +
                           (target_e - start_e) ** 2 +
                           (target_d - start_d) ** 2)

    # A leg that is mostly vertical is a level change, and is flown slowly.
    vertical_span = abs(target_d - start_d)
    horizontal_span = math.sqrt((target_n - start_n) ** 2 +
                                (target_e - start_e) ** 2)
    is_climb = vertical_span > horizontal_span
    speed = CLIMB_SPEED if is_climb else CRUISE_SPEED

    kind = "climb" if is_climb else "cruise"
    print(f"\n[WAYPOINT {index}/{total}] x={x:+.1f} y={y:+.1f} z={z:.2f} "
          f"heading {yaw_deg:+.0f}  {leg_length:.1f} m  {kind} at {speed} m/s")

    dt = 0.1
    travelled = 0.0
    elapsed = 0.0
    max_time = leg_length / speed + TIMEOUT_MARGIN

    # Altitude hold measures clean while stationary (3 mm bias, 1 cm spread),
    # so any vertical disturbance must come from the motion itself. Track it
    # per leg to see whether that is what pushes codes out of frame.
    alt_samples = []

    while elapsed < max_time:
        travelled = min(leg_length, travelled + speed * dt)
        fraction = 1.0 if leg_length == 0 else travelled / leg_length
        cn, ce = corrected(start_n + (target_n - start_n) * fraction,
                           start_e + (target_e - start_e) * fraction)
        await drone.offboard.set_position_ned(PositionNedYaw(
            cn, ce, start_d + (target_d - start_d) * fraction, yaw_deg))
        await poll_camera()

        alt_samples.append(-current_pos["d"] - z)

        if (travelled >= leg_length
                and distance_to(target_n, target_e, target_d) < WAYPOINT_TOLERANCE):
            # Hold until the vehicle has actually caught up. Moving on the
            # moment the setpoint arrives leaves residual error that carries
            # into the next leg, which is what pushed codes out of frame.
            settle = 0.0
            while settle < SETTLE_TIMEOUT_S:
                cn, ce = corrected(target_n, target_e)
                await drone.offboard.set_position_ned(PositionNedYaw(
                    cn, ce, target_d, yaw_deg))
                await poll_camera()
                if abs(-current_pos["d"] - z) < SETTLE_TOLERANCE:
                    break
                await asyncio.sleep(dt)
                settle += dt
                
            # Wait an extra 1.5 seconds for the drone to level out its roll/pitch 
            # after braking. We poll the camera during this time because the drone is 
            # flat and hovering, which gives us perfectly accurate ArUco drift readings.
            global is_settled
            is_settled = True
            elapsed_settle = 0.0
            while elapsed_settle < 1.5:
                await poll_camera()
                await asyncio.sleep(dt)
                elapsed_settle += dt
            is_settled = False

            residual = -current_pos["d"] - z
            if alt_samples:
                worst = max(alt_samples, key=abs)
                spread = max(alt_samples) - min(alt_samples)
                flag = "  OUT OF FRAME" if abs(worst) > 0.37 else ""
                print(f"           reached  codes {len(inventory)}  "
                      f"alt worst {worst:+.3f} spread {spread:.3f} "
                      f"residual {residual:+.3f} settled in {settle:.1f}s{flag}")
            return True

        await asyncio.sleep(dt)
        elapsed += dt

    print(f"           timeout, remaining "
          f"{distance_to(target_n, target_e, target_d):.2f} m")
    return False


async def run():
    route = build_route()
    print("=" * 68)
    print("  AUTONOMOUS WAREHOUSE INVENTORY SCAN")
    print("  Sensor configuration: C27, single front-facing scanning camera")
    print(f"  Shelf faces    : {len(ISLAND_X) * 2}")
    print(f"  Shelf levels   : {len(FLIGHT_Z)}")
    print(f"  Waypoints      : {len(route)}")
    print("  Localization   : visual odometry with EKF2, GPS disabled")
    print("  Drift correction: ArUco floor markers, offset applied to setpoints")
    print("=" * 68)

    load_marker_map()

    cam_node = trans.Node()
    cam_node.subscribe(Image, CAM_HIRES, on_hires_image)
    down_node = trans.Node()
    down_node.subscribe(Image, CAM_DOWN, on_down_image)

    drone = System()
    await drone.connect(system_address="udp://:14540")
    print("\n[INFO] Connecting to vehicle")
    async for state in drone.core.connection_state():
        if state.is_connected:
            break

    asyncio.create_task(track_position(drone))
    asyncio.create_task(track_heading(drone))

    print("[INFO] Waiting for position estimate (VIO Local Position)")
    async for health in drone.telemetry.health():
        if health.is_local_position_ok:
            break
    print("[INFO] Position estimate valid")

    await asyncio.sleep(2)  # Ensure current_pos is populated
    global origin_pos
    origin_pos = {"n": current_pos["n"], "e": current_pos["e"], "d": current_pos["d"]}
    print(f"[INFO] EKF2 Local Origin mapped to: {origin_pos}")

    print("[INFO] Arming and taking off (Offboard VIO Mode)")
    # Hold current position and heading to prevent violent spin on the ground
    start_n = current_pos["n"]
    start_e = current_pos["e"]
    start_d = current_pos["d"]
    start_yaw = current_yaw["deg"]
    
    await drone.offboard.set_position_ned(PositionNedYaw(
        start_n, start_e, start_d, start_yaw))
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"[ERROR] Offboard rejected: {error}")
        return
        
    await drone.action.arm()
    
    # Smoothly ascend to 1.5m above ground
    print("           ascending...")
    for i in range(15):
        target_d = start_d - (i / 10.0)
        await drone.offboard.set_position_ned(PositionNedYaw(
            start_n, start_e, target_d, start_yaw))
        await asyncio.sleep(0.5)
        
    print("           turning to route heading...")
    await hold_heading(drone, YAW_NORTH)

    print("\n[INFO] Starting scan")
    reached = 0
    last_yaw = YAW_NORTH
    for index, (x, y, z, yaw) in enumerate(route, 1):
        if yaw != last_yaw:
            await hold_heading(drone, yaw)
            last_yaw = yaw
        if await goto_waypoint(drone, index, len(route), x, y, z, yaw):
            reached += 1

    print("\n" + "=" * 68)
    print("  SCAN COMPLETE")
    print(f"  Waypoints reached : {reached}/{len(route)}")
    print(f"  Codes decoded     : {len(inventory)}")
    print(f"  Marker fixes      : {len(marker_events)}")
    if marker_events:
        drifts = [e["error_m"] for e in marker_events]
        drifts_sorted = sorted(drifts)
        median = drifts_sorted[len(drifts_sorted) // 2]
        print(f"  Drift at fix      : median {median:.3f} m, "
              f"max {max(drifts):.3f} m")
        print(f"  Final offset      : "
              f"{math.hypot(drift_offset['n'], drift_offset['e']):.3f} m")
    print("=" * 68)

    with open(OUTPUT_JSON, "w", encoding="utf-8") as handle:
        json.dump({
            "scan_date": datetime.now().isoformat(timespec="seconds"),
            "sensor_configuration": "C27, single front-facing scanning camera",
            "localization": "visual odometry, no GPS",
            "total_detected": len(inventory),
            "waypoints_completed": f"{reached}/{len(route)}",
            "marker_corrections": marker_events,
            "final_drift_offset_m": round(
                math.hypot(drift_offset["n"], drift_offset["e"]), 3),
            "items": list(inventory.values()),
        }, handle, indent=2, ensure_ascii=False)
    print(f"\n[INFO] Inventory written to {OUTPUT_JSON}")

    print("[INFO] Landing")
    await drone.offboard.stop()
    await drone.action.land()


if __name__ == "__main__":
    asyncio.run(run())
