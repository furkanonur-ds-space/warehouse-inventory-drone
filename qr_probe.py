"""
Diagnose QR decode performance at a standstill.

Flies to a set of positions where a box is known to be directly beside the
vehicle, holds still, and reports whether the code decodes. Because the vehicle
is stationary, this separates a readability problem (size, resolution, contrast)
from a motion problem (dropped frames, blur).

Also reports the measured pixel size of the code, so the readability budget can
be checked against reality rather than assumed.

Usage:
    python3 qr_probe.py
"""
import os
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"

import asyncio
import json
import math
import cv2
import numpy as np
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError
import gz.transport13 as trans
from gz.msgs10.image_pb2 import Image

# Must match build_v2_world.py
CORRIDOR_X = [-6.0, -2.0, 2.0, 6.0]
ISLAND_X = [-4.0, 0.0, 4.0]
FLIGHT_Z = [0.5, 1.15, 1.8]
BOX_Y = [-4.0, 0.0, 4.0]
SPAWN_X, SPAWN_Y = -6.0, -9.0

# Heading is held at +90 degrees throughout.
#
# The vehicle spawns with a yaw of 1.5708 rad so that it faces north, along the
# aisle, which puts the side cameras perpendicular to the shelf faces. Sending
# a yaw setpoint of 0 makes it rotate, after which the side cameras look along
# the aisle instead of at the shelf, and nothing decodes. This was diagnosed
# from saved frames: aisle 1 showed a shelf face square-on, aisle 2 showed the
# shelf receding into perspective.
FLIGHT_YAW_DEG = 90.0


WORLD = "warehouse_v2"
DRONE = "x500_scanner_0"
CAM_LEFT = f"/world/{WORLD}/model/{DRONE}/link/camera_left_link/sensor/camera/image"
CAM_RIGHT = f"/world/{WORLD}/model/{DRONE}/link/camera_right_link/sensor/camera/image"

OUT_DIR = os.path.expanduser("~/autonomous_landing/qr_probe_frames")
os.makedirs(OUT_DIR, exist_ok=True)

current_pos = {"n": 0.0, "e": 0.0, "d": 0.0}
frames = {"left": None, "right": None}
qr_detector = cv2.QRCodeDetector()


def on_left(msg):
    try:
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            (msg.height, msg.width, 3))
        frames["left"] = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    except Exception:
        pass


def on_right(msg):
    try:
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            (msg.height, msg.width, 3))
        frames["right"] = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    except Exception:
        pass


async def track(drone):
    try:
        async for odom in drone.telemetry.position_velocity_ned():
            current_pos["n"] = odom.position.north_m
            current_pos["e"] = odom.position.east_m
            current_pos["d"] = odom.position.down_m
    except Exception:
        pass


def analyse(frame, tag, side):
    """Try to decode, and measure how large the code appears in pixels."""
    if frame is None:
        return {"side": side, "status": "no frame"}

    result = {"side": side}
    cv2.imwrite(os.path.join(OUT_DIR, f"{tag}_{side}.png"), frame)

    ok, points = qr_detector.detectMulti(frame)
    if not ok or points is None:
        ok, points = qr_detector.detect(frame)
        if ok and points is not None:
            points = points
        else:
            result["status"] = "not detected"
            return result

    quad = points[0]
    p = quad.astype(int)
    width_px = p[:, 0].max() - p[:, 0].min()
    height_px = p[:, 1].max() - p[:, 1].min()
    side_px = max(width_px, height_px)
    result["code_px"] = int(side_px)
    result["px_per_module"] = round(side_px / 29.0, 2)   # version 3 plus quiet zone

    x1 = max(0, p[:, 0].min() - 8)
    y1 = max(0, p[:, 1].min() - 8)
    x2 = min(frame.shape[1], p[:, 0].max() + 8)
    y2 = min(frame.shape[0], p[:, 1].max() + 8)
    crop = frame[y1:y2, x1:x2]

    if crop.size == 0:
        result["status"] = "detected, empty crop"
        return result

    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    result["contrast_min"] = int(gray.min())
    result["contrast_max"] = int(gray.max())

    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    data, _, _ = qr_detector.detectAndDecode(binary)
    if data:
        result["status"] = "decoded"
        result["value"] = data
        return result

    upscaled = cv2.resize(binary, None, fx=3.0, fy=3.0,
                          interpolation=cv2.INTER_CUBIC)
    data_up, _, _ = qr_detector.detectAndDecode(upscaled)
    if data_up:
        result["status"] = "decoded after upscale"
        result["value"] = data_up
        return result

    result["status"] = "detected but not decoded"
    return result


async def goto(drone, x, y, z):
    target_n = y - SPAWN_Y
    target_e = x - SPAWN_X
    target_d = -z
    for _ in range(200):
        await drone.offboard.set_position_ned(
            PositionNedYaw(target_n, target_e, target_d, FLIGHT_YAW_DEG))
        dn = target_n - current_pos["n"]
        de = target_e - current_pos["e"]
        dd = target_d - current_pos["d"]
        if math.sqrt(dn*dn + de*de + dd*dd) < 0.25:
            return True
        await asyncio.sleep(0.15)
    return False


async def run():
    left_node = trans.Node()
    left_node.subscribe(Image, CAM_LEFT, on_left)
    right_node = trans.Node()
    right_node.subscribe(Image, CAM_RIGHT, on_right)

    drone = System()
    await drone.connect(system_address="udp://:14540")
    print("[INFO] Connecting")
    async for state in drone.core.connection_state():
        if state.is_connected:
            break

    asyncio.create_task(track(drone))

    async for health in drone.telemetry.health():
        if health.is_local_position_ok and health.is_home_position_ok:
            break
    print("[INFO] Arming")

    await drone.action.arm()
    await drone.action.takeoff()
    await asyncio.sleep(8)

    await drone.offboard.set_position_ned(PositionNedYaw(
        current_pos["n"], current_pos["e"], current_pos["d"], FLIGHT_YAW_DEG))
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"[ERROR] Offboard rejected: {error}")
        await drone.action.land()
        return

    # Test every level at one Y position in the first two aisles.
    tests = []
    for corridor in CORRIDOR_X[:2]:
        for z in FLIGHT_Z:
            tests.append((corridor, 0.0, z))

    print(f"\n[INFO] {len(tests)} stationary probes\n")
    print(f"{'position':26s} {'side':6s} {'status':28s} {'px':>5s} {'px/mod':>7s} {'contrast':>10s}")
    print("-" * 92)

    results = []
    for x, y, z in tests:
        await goto(drone, x, y, z)
        await asyncio.sleep(2.5)   # let the image settle

        tag = f"x{x:+.1f}_y{y:+.1f}_z{z:.2f}".replace(".", "p").replace("+", "")
        label = f"x={x:+.1f} y={y:+.1f} z={z:.2f}"

        for side in ("left", "right"):
            r = analyse(frames[side], tag, side)
            r["position"] = {"x": x, "y": y, "z": z}
            results.append(r)

            px = r.get("code_px", "")
            ppm = r.get("px_per_module", "")
            contrast = ""
            if "contrast_min" in r:
                contrast = f"{r['contrast_min']}-{r['contrast_max']}"
            print(f"{label:26s} {side:6s} {r['status']:28s} "
                  f"{str(px):>5s} {str(ppm):>7s} {contrast:>10s}")

    decoded = sum(1 for r in results
                  if r["status"].startswith("decoded"))
    detected = sum(1 for r in results
                   if r["status"] != "not detected" and r["status"] != "no frame")

    print("\n" + "=" * 92)
    print(f"  Probes            : {len(results)}")
    print(f"  Code detected     : {detected}")
    print(f"  Code decoded      : {decoded}")
    print("=" * 92)
    print("\n  If codes are detected but not decoded, the problem is readability")
    print("  (size, resolution or contrast), not motion. Check px/module against")
    print("  the threshold of about 3.0.")

    out = os.path.expanduser("~/autonomous_landing/qr_probe_results.json")
    with open(out, "w", encoding="utf-8") as handle:
        json.dump(results, handle, indent=2, ensure_ascii=False)
    print(f"\n[INFO] Results written to {out}")
    print(f"[INFO] Frames saved in {OUT_DIR}")

    await drone.offboard.stop()
    await drone.action.land()


if __name__ == "__main__":
    asyncio.run(run())
