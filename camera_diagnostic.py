"""
TANI: Kameralar gercekte ne goruyor?
Holds the vehicle stationary directly in front of a single box and records
from both cameras, so an alignment problem can be seen rather than guessed.
"""
import os
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"

import asyncio
import cv2
import numpy as np
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError
import gz.transport13 as trans
from gz.msgs10.image_pb2 import Image

WORLD = "warehouse_v2"
DRONE = "x500_scanner_0"
CAM_LEFT  = f"/world/{WORLD}/model/{DRONE}/link/camera_left_link/sensor/camera/image"
CAM_RIGHT = f"/world/{WORLD}/model/{DRONE}/link/camera_right_link/sensor/camera/image"

SPAWN_X, SPAWN_Y = -8.5, -9.0
OUT_DIR = os.path.expanduser("~/autonomous_landing/cam_debug")
os.makedirs(OUT_DIR, exist_ok=True)

frame_left = None
frame_right = None
qr_detector = cv2.QRCodeDetector()
current_pos = {"n": 0.0, "e": 0.0, "d": 0.0}


def on_left(msg):
    global frame_left
    try:
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        frame_left = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    except Exception:
        pass


def on_right(msg):
    global frame_right
    try:
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        frame_right = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    except Exception:
        pass


async def track(drone):
    try:
        async for o in drone.telemetry.position_velocity_ned():
            current_pos["n"] = o.position.north_m
            current_pos["e"] = o.position.east_m
            current_pos["d"] = o.position.down_m
    except Exception:
        pass


def save_and_check(tag):
    """Save a frame from both cameras and check whether a QR code is present."""
    results = {}
    for name, frame in [("sol", frame_left), ("sag", frame_right)]:
        if frame is None:
            print(f"    {name}: NO IMAGE")
            results[name] = None
            continue
        data, pts, _ = qr_detector.detectAndDecode(frame)
        path = os.path.join(OUT_DIR, f"{tag}_{name}.png")
        annotated = frame.copy()
        if data:
            print(f"    {name}: QR OKUNDU -> {data}")
            if pts is not None:
                cv2.polylines(annotated, [pts.astype(int)], True, (0,255,0), 3)
        else:
            print(f"    {name}: no QR code (frame saved)")
        cv2.imwrite(path, annotated)
        results[name] = data
    return results


async def run():
    node_l = trans.Node(); node_l.subscribe(Image, CAM_LEFT, on_left)
    node_r = trans.Node(); node_r.subscribe(Image, CAM_RIGHT, on_right)

    drone = System()
    await drone.connect(system_address="udp://:14540")
    print("[INFO] Connecting...")
    async for s in drone.core.connection_state():
        if s.is_connected: break

    asyncio.create_task(track(drone))

    async for h in drone.telemetry.health():
        if h.is_local_position_ok and h.is_home_position_ok: break
    print("[INFO] Sensors ready. Taking off...")

    await drone.action.arm()
    await drone.action.takeoff()
    await asyncio.sleep(8)

    await drone.offboard.set_position_ned(
        PositionNedYaw(current_pos["n"], current_pos["e"], current_pos["d"], 0.0))
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"[ERROR] {e}"); await drone.action.land(); return

    # Known box positions: island 1 (x=-6.2), left-face boxes around x=-6.85
    # Seen from aisle 1 (x=-8.5), the boxes are on the right
    # Y positions: -4, 0, 4  |  Z levels: 0.55, 1.2, 1.85 (box centres)
    test_points = [
        # (aciklama, drone_x, drone_y, drone_z)
        ("k1_alt",   -8.5, -4.0, 0.55),
        ("k1_orta",  -8.5, -4.0, 1.20),
        ("k1_ust",   -8.5, -4.0, 1.85),
        ("k2_orta",  -8.5,  0.0, 1.20),
        ("k3_orta",  -8.5,  4.0, 1.20),
        # View from aisle 2 (island 1 right and island 2 left should both be visible)
        ("kor2_orta", -3.9, 0.0, 1.20),
        ("kor2_alt",  -3.9, -4.0, 0.55),
    ]

    for tag, gx, gy, gz in test_points:
        tn = gy - SPAWN_Y
        te = gx - SPAWN_X
        td = -gz
        print(f"\n[TEST {tag}] Gazebo(x={gx}, y={gy}, z={gz}) konumuna gidiliyor...")

        for _ in range(120):   # max 18 saniye
            await drone.offboard.set_position_ned(PositionNedYaw(tn, te, td, 0.0))
            d = ((tn-current_pos["n"])**2 + (te-current_pos["e"])**2 + (td-current_pos["d"])**2) ** 0.5
            if d < 0.3:
                break
            await asyncio.sleep(0.15)

        await asyncio.sleep(2.0)   # let the image settle
        print(f"  Konumda. Kameralar kontrol ediliyor:")
        save_and_check(tag)

    print(f"\n[INFO] Frames saved to: {OUT_DIR}")
    print("[BILGI] Iniliyor...")
    await drone.offboard.stop()
    await drone.action.land()


if __name__ == "__main__":
    asyncio.run(run())
