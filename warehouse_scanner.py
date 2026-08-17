"""
Autonomous Warehouse Inventory Scanner
======================================

Scans a warehouse using a UAV equipped with two side-facing cameras and an
optical flow sensor. No GPS is used at any point - localization relies on
optical flow dead reckoning fused by the PX4 EKF2 estimator.

The warehouse floor plan is known in advance, so the flight path is a
pre-planned boustrophedon (zigzag) route rather than reactive exploration.

Outputs a JSON inventory file mapping every detected QR code to an estimated
3D position within the warehouse.
"""
import os
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"

import asyncio
import math
import json
import cv2
import numpy as np
from datetime import datetime
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError
import gz.transport13 as trans
from gz.msgs10.image_pb2 import Image

# --- WAREHOUSE FLOOR PLAN (known in advance) ---------------------------
CORRIDOR_X = [-8.5, -3.9, 0.7, 5.3]     # corridor centre lines
FLIGHT_Z = [0.5, 1.15, 1.8]              # flight altitudes, one per shelf level
Y_SOUTH, Y_NORTH = -8.0, 8.0             # corridor end points
SPAWN_X, SPAWN_Y = -8.5, -9.0            # UAV spawn point (NED origin)

# --- SCAN PARAMETERS ---------------------------------------------------
WAYPOINT_TOLERANCE = 0.4     # metres - waypoint considered reached below this
CRUISE_SPEED = 0.7            # m/s - setpoint advance rate along each leg
TIMEOUT_MARGIN = 15.0         # seconds of slack added to the expected leg time

# --- GAZEBO TOPICS -----------------------------------------------------
WORLD = "warehouse_v2"
DRONE = "x500_scanner_0"
CAM_LEFT = f"/world/{WORLD}/model/{DRONE}/link/camera_left_link/sensor/camera/image"
CAM_RIGHT = f"/world/{WORLD}/model/{DRONE}/link/camera_right_link/sensor/camera/image"

OUTPUT_JSON = os.path.expanduser("~/autonomous_landing/inventory_scanned.json")

# --- STATE -------------------------------------------------------------
current_pos = {"n": 0.0, "e": 0.0, "d": 0.0}
detected_left = []
detected_right = []
inventory = {}

qr_detector = cv2.QRCodeDetector()


def ned_to_gazebo(n, e, d):
    """Convert a NED coordinate back into Gazebo world coordinates."""
    return e + SPAWN_X, n + SPAWN_Y, -d


def decode_qr(frame):
    """
    Decode QR codes using a detect-crop-threshold pipeline.

    Gazebo's lighting model renders the white quiet zone of a QR code as mid
    grey, which leaves too little contrast for the decoder to work on the raw
    frame. Applying Otsu thresholding to the whole frame does not help either,
    because different regions of the frame have different brightness.

    The reliable approach is: locate the QR quad first, crop that region only,
    then apply Otsu thresholding locally. A 3x upscale is used as a fallback
    for codes seen at longer range.
    """
    results = []
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
            data, _, _ = qr_detector.detectAndDecode(binary)
            if data:
                results.append(data)
                continue

            upscaled = cv2.resize(binary, None, fx=3.0, fy=3.0,
                                  interpolation=cv2.INTER_CUBIC)
            data_up, _, _ = qr_detector.detectAndDecode(upscaled)
            if data_up:
                results.append(data_up)
    except Exception:
        pass
    return results


def on_image_left(msg):
    global detected_left
    try:
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            (msg.height, msg.width, 3))
        detected_left = decode_qr(cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
    except Exception:
        pass


def on_image_right(msg):
    global detected_right
    try:
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            (msg.height, msg.width, 3))
        detected_right = decode_qr(cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
    except Exception:
        pass


async def track_position(drone):
    """Continuously mirror the EKF2 local position estimate into current_pos."""
    global current_pos
    try:
        async for odom in drone.telemetry.position_velocity_ned():
            current_pos["n"] = odom.position.north_m
            current_pos["e"] = odom.position.east_m
            current_pos["d"] = odom.position.down_m
    except Exception:
        pass


def record_detection(qr_id, side):
    """Store a newly seen QR code together with an estimated shelf position."""
    if qr_id in inventory:
        return False

    gx, gy, gz = ned_to_gazebo(current_pos["n"], current_pos["e"],
                               current_pos["d"])

    # The box sits on the shelf face the camera is pointing at, roughly 0.8 m
    # to the side of the UAV.
    lateral_offset = -0.8 if side == "left" else 0.8

    inventory[qr_id] = {
        "id": qr_id,
        "detected_by": side,
        "estimated_x": round(gx + lateral_offset, 2),
        "estimated_y": round(gy, 2),
        "estimated_z": round(gz - 0.15, 2),
        "uav_position": {"x": round(gx, 2), "y": round(gy, 2), "z": round(gz, 2)},
        "timestamp": datetime.now().isoformat(timespec="seconds"),
    }
    print(f"           DETECTED [{side.upper():5s}] {qr_id}")
    print(f"                    estimated position "
          f"x={gx + lateral_offset:.2f} y={gy:.2f} z={gz - 0.15:.2f}")
    return True


async def poll_cameras():
    """Check both cameras and record any newly decoded codes."""
    for qr in detected_left:
        record_detection(qr, "left")
    for qr in detected_right:
        record_detection(qr, "right")


def build_route():
    """
    Build a boustrophedon route over the whole warehouse.

    Each corridor is traversed once per shelf level. The traversal direction
    alternates so the UAV never has to fly back to the start of a corridor.
    """
    route = []
    heading_north = True
    for corridor_index, x in enumerate(CORRIDOR_X):
        levels = FLIGHT_Z if corridor_index % 2 == 0 else list(reversed(FLIGHT_Z))
        for z in levels:
            if heading_north:
                route.append((x, Y_SOUTH, z))
                route.append((x, Y_NORTH, z))
            else:
                route.append((x, Y_NORTH, z))
                route.append((x, Y_SOUTH, z))
            heading_north = not heading_north
    return route


def distance_to(target_n, target_e, target_d):
    dn = target_n - current_pos["n"]
    de = target_e - current_pos["e"]
    dd = target_d - current_pos["d"]
    return math.sqrt(dn * dn + de * de + dd * dd)


async def goto_waypoint(drone, index, total, x, y, z):
    """
    Fly to a waypoint using a moving setpoint ("carrot") that advances at a
    constant rate along the straight line to the target.

    Advancing the setpoint on a timer rather than waiting for the vehicle to
    reach each intermediate point avoids the stop-start motion that a
    tolerance-gated stepper produces, so the vehicle cruises smoothly and the
    camera sees each box across a steady sequence of frames.
    """
    target_n = y - SPAWN_Y
    target_e = x - SPAWN_X
    target_d = -z

    start_n = current_pos["n"]
    start_e = current_pos["e"]
    start_d = current_pos["d"]
    leg_length = math.sqrt((target_n - start_n) ** 2 +
                           (target_e - start_e) ** 2 +
                           (target_d - start_d) ** 2)

    print(f"\n[WAYPOINT {index}/{total}] target x={x:.1f} y={y:.1f} z={z:.2f} "
          f"({leg_length:.1f} m)")

    dt = 0.1
    travelled = 0.0
    elapsed = 0.0
    max_time = leg_length / CRUISE_SPEED + TIMEOUT_MARGIN

    while elapsed < max_time:
        # Advance the carrot along the leg at a constant speed.
        travelled = min(leg_length, travelled + CRUISE_SPEED * dt)
        fraction = 1.0 if leg_length == 0 else travelled / leg_length
        carrot_n = start_n + (target_n - start_n) * fraction
        carrot_e = start_e + (target_e - start_e) * fraction
        carrot_d = start_d + (target_d - start_d) * fraction

        await drone.offboard.set_position_ned(
            PositionNedYaw(carrot_n, carrot_e, carrot_d, 0.0))
        await poll_cameras()

        if (travelled >= leg_length
                and distance_to(target_n, target_e, target_d) < WAYPOINT_TOLERANCE):
            print(f"           reached (codes so far: {len(inventory)})")
            return True

        await asyncio.sleep(dt)
        elapsed += dt

    print(f"           timeout, remaining distance "
          f"{distance_to(target_n, target_e, target_d):.2f} m")
    return False


async def run():
    route = build_route()
    print("=" * 65)
    print("  AUTONOMOUS WAREHOUSE INVENTORY SCAN")
    print(f"  Corridors      : {len(CORRIDOR_X)}")
    print(f"  Shelf levels   : {len(FLIGHT_Z)}")
    print(f"  Waypoints      : {len(route)}")
    print("  Localization   : optical flow + EKF2 (GPS disabled)")
    print("=" * 65)

    left_node = trans.Node()
    left_node.subscribe(Image, CAM_LEFT, on_image_left)
    right_node = trans.Node()
    right_node.subscribe(Image, CAM_RIGHT, on_image_right)

    drone = System()
    await drone.connect(system_address="udp://:14540")
    print("\n[INFO] Connecting to vehicle")
    async for state in drone.core.connection_state():
        if state.is_connected:
            break

    asyncio.create_task(track_position(drone))

    print("[INFO] Waiting for position estimate")
    async for health in drone.telemetry.health():
        if health.is_local_position_ok and health.is_home_position_ok:
            break
    print("[INFO] Position estimate valid")

    print("[INFO] Arming and taking off")
    await drone.action.arm()
    await drone.action.takeoff()
    await asyncio.sleep(8)

    print("[INFO] Entering offboard mode")
    await drone.offboard.set_position_ned(
        PositionNedYaw(current_pos["n"], current_pos["e"], current_pos["d"], 0.0))
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"[ERROR] Offboard rejected: {error}")
        await drone.action.land()
        return

    print("\n[INFO] Starting scan")
    reached = 0
    for index, (x, y, z) in enumerate(route, 1):
        if await goto_waypoint(drone, index, len(route), x, y, z):
            reached += 1

    print("\n" + "=" * 65)
    print("  SCAN COMPLETE")
    print(f"  Waypoints reached : {reached}/{len(route)}")
    print(f"  Codes decoded     : {len(inventory)}")
    print("=" * 65)

    with open(OUTPUT_JSON, "w", encoding="utf-8") as handle:
        json.dump({
            "scan_date": datetime.now().isoformat(timespec="seconds"),
            "localization": "optical flow (no GPS)",
            "total_detected": len(inventory),
            "waypoints_completed": f"{reached}/{len(route)}",
            "items": list(inventory.values()),
        }, handle, indent=2, ensure_ascii=False)
    print(f"\n[INFO] Inventory written to {OUTPUT_JSON}")

    print("[INFO] Landing")
    await drone.offboard.stop()
    await drone.action.land()


if __name__ == "__main__":
    asyncio.run(run())
