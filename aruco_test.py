import os
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"

import asyncio
import cv2
import numpy as np
import csv
import time
import sys
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed, OffboardError
import gz.transport13 as trans
from gz.msgs10.image_pb2 import Image

# --- SETTINGS ----------------------------------------------
MARKER_ID   = int(sys.argv[1])   if len(sys.argv) > 1 else 0
MARKER_SIZE = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
LIGHT_COND  = sys.argv[3]        if len(sys.argv) > 3 else "normal"
CSV_FILE    = os.path.expanduser("~/autonomous_landing/aruco_test_results.csv")

WORLD_NAME   = f"test_id{MARKER_ID}_s{str(MARKER_SIZE).replace('.','p')}_{LIGHT_COND}"
CAMERA_TOPIC = f"/world/{WORLD_NAME}/model/x500_0/link/base_link/sensor/camera/image"
ARUCO_DICT   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
ARUCO_PARAMS = cv2.aruco.DetectorParameters()
DETECTOR     = cv2.aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)

# --- STATE -------------------------------------------------
latest_frame     = None
current_altitude = 0.0
sensors_ready    = False
detected         = False
err_x, err_y     = 0, 0

# --- CAMERA ------------------------------------------------
def on_image(msg):
    global latest_frame, detected, err_x, err_y
    try:
        img   = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        frame = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        fh, fw = frame.shape[:2]

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = DETECTOR.detectMarkers(gray)

        detected = False
        if ids is not None and len(ids) > 0:
            if MARKER_ID in ids.flatten():
                idx = list(ids.flatten()).index(MARKER_ID)
                c   = corners[idx][0]
                cx  = int(np.mean(c[:, 0]))
                cy  = int(np.mean(c[:, 1]))
                err_x = cx - fw // 2
                err_y = cy - fh // 2
                detected = True
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        cv2.imwrite(f"/home/furk/autonomous_landing/debug_test_id{MARKER_ID}.png", frame)
        latest_frame = frame
    except Exception:
        pass

# --- BACKGROUND --------------------------------------------
async def get_altitude(drone):
    global current_altitude
    try:
        async for pos in drone.telemetry.position():
            current_altitude = pos.relative_altitude_m
    except: pass

async def get_health(drone):
    global sensors_ready
    try:
        async for h in drone.telemetry.health():
            sensors_ready = (h.is_global_position_ok and
                             h.is_local_position_ok  and
                             h.is_home_position_ok)
    except: pass

# --- CSV OUTPUT --------------------------------------------
def save_result(altitude, yaw_deg, det, ex, ey, duration_ms):
    file_exists = os.path.isfile(CSV_FILE)
    with open(CSV_FILE, "a", newline="") as f:
        writer = csv.writer(f)
        if not file_exists:
            writer.writerow([
                "marker_id", "marker_size_m", "light_condition",
                "altitude_m", "yaw_angle_deg",
                "detected", "err_x_px", "err_y_px", "detect_time_ms"
            ])
        writer.writerow([
            MARKER_ID, MARKER_SIZE, LIGHT_COND,
            round(altitude, 2), yaw_deg,
            int(det), ex, ey, round(duration_ms, 1)
        ])
    status = "DETECTED" if det else "NOT DETECTED"
    print(f"  [{status}] Alt={altitude:.1f}m  Yaw={yaw_deg}deg  "
          f"Error=({ex},{ey})px  Time={duration_ms:.0f}ms")

# --- TEST SCENARIOS ----------------------------------------
# (altitude_m, yaw_angle_degrees)
TEST_SCENARIOS = [
    (1.0,  0),   # 1 m altitude, looking straight down
    (2.0,  0),   # 2 m altitude, looking straight down
    (5.0,  0),   # 5 m altitude, looking straight down
    (2.0, 30),   # 2 m altitude, 30 degree yaw
    (2.0, 45),   # 2 m altitude, 45 degree yaw
    (2.0, 60),   # 2 m altitude, 60 degree yaw
]

# --- MAIN --------------------------------------------------
async def run():
    print(f"\n{'='*55}")
    print(f"  ARUCO TEST STARTING")
    print(f"  Marker ID   : {MARKER_ID}")
    print(f"  Marker size : {MARKER_SIZE}m x {MARKER_SIZE}m")
    print(f"  Isik Kosulu : {LIGHT_COND}")
    print(f"  Senaryo Say.: {len(TEST_SCENARIOS)}")
    print(f"{'='*55}\n")

    node = trans.Node()
    node.subscribe(Image, CAMERA_TOPIC, on_image)

    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("[INFO] Waiting for vehicle connection...")
    async for state in drone.core.connection_state():
        if state.is_connected: break

    asyncio.create_task(get_altitude(drone))
    asyncio.create_task(get_health(drone))

    print("[INFO] Waiting for sensors...")
    while not sensors_ready:
        await asyncio.sleep(0.2)

    print("[INFO] Taking off...")
    await drone.action.arm()
    await drone.action.takeoff()
    while current_altitude < 1.8:
        await asyncio.sleep(0.1)

    print(f"[INFO] Altitude {current_altitude:.1f}m. Switching to offboard...")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"[ERROR] Offboard: {e}")
        return

    # --- TEST LOOP -----------------------------------------
    for i, (target_alt, yaw_deg) in enumerate(TEST_SCENARIOS):
        print(f"\n[TEST {i+1}/{len(TEST_SCENARIOS)}] "
              f"Target altitude={target_alt}m  Yaw={yaw_deg}deg")

        # 1) Climb or descend to the target altitude
        current = current_altitude
        vz = -0.5 if target_alt > current_altitude else 0.5
        while abs(current_altitude - target_alt) > 0.15:
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0.0, 0.0, vz, 0.0))
            await asyncio.sleep(0.1)
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(1.5)  # stabilize

        # 2) Apply yaw
        if yaw_deg > 0:
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0.0, 0.0, 0.0, 30.0))
            await asyncio.sleep(yaw_deg / 30.0)
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await asyncio.sleep(1.0)

        # 3) Measure detection for 3 seconds
        results = []
        t_start = time.time()
        for _ in range(30):   # 30 x 0.1 s = 3 seconds
            t0  = time.time()
            det = detected
            ex, ey = err_x, err_y
            dt = (time.time() - t0) * 1000
            results.append((det, ex, ey, dt))
            await asyncio.sleep(0.1)

        # 4) Summarise and record the result
        det_count  = sum(1 for r in results if r[0])
        det_rate   = det_count / len(results) * 100
        best       = next(((ex, ey, dt) for (d, ex, ey, dt) in results if d), (0, 0, 0))
        avg_dt     = np.mean([r[3] for r in results])

        print(f"  Detection rate: {det_rate:.0f}%  ({det_count}/30 frames)")
        save_result(current_altitude, yaw_deg,
                    det_rate > 50, best[0], best[1], avg_dt)

        # 5) Return yaw to zero
        if yaw_deg > 0:
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0.0, 0.0, 0.0, -30.0))
            await asyncio.sleep(yaw_deg / 30.0)
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await asyncio.sleep(0.5)

    # --- LANDING -------------------------------------------
    print("\n[INFO] Tests complete. Landing...")
    await drone.offboard.stop()
    await drone.action.land()

    print(f"\n{'='*55}")
    print(f"  ALL TESTS COMPLETE")
    print(f"  Results: {CSV_FILE}")
    print(f"{'='*55}\n")

if __name__ == "__main__":
    asyncio.run(run())
