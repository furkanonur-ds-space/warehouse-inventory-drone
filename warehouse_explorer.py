import os
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"

import asyncio
import cv2
import numpy as np
import socket
import sys
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed, OffboardError
import gz.transport13 as trans
from gz.msgs10.laserscan_pb2 import LaserScan
from gz.msgs10.image_pb2 import Image

# ─── AYARLAR ────────────────────────────────────────────────
LIDAR_TOPIC  = "/world/warehouse/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan"
CAMERA_TOPIC = "/world/warehouse/model/x500_lidar_2d_0/link/base_link/sensor/camera/image"
TARGET_QR    = sys.argv[1] if len(sys.argv) > 1 else "RAF-A1-KUTU-001"

EMERGENCY_SIDE_DIST = 0.6
FALLBACK_FRONT_DIST = 1.0
FORWARD_SPEED       = 0.5
TURN_SPEED          = 25.0
SIDE_SHIFT_DURATION = 5.0
STRAFE_SPEED        = 0.4
MAX_PASS_DURATION   = 45.0

END_MARKER_IDS = [2, 4, 6]
NUM_CORRIDORS  = len(END_MARKER_IDS)

# ─── UDP ISTIHBARAT ─────────────────────────────────────────
udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def send_intel(msg):
    full = f"[DEPO-DRONE] {msg}"
    udp_sock.sendto(full.encode("utf-8"), ("127.0.0.1", 5005))
    print(full)


# ─── DURUM DEGISKENLERI ─────────────────────────────────────
latest_ranges = None
current_altitude = 0.0
sensors_ready = False

qr_detected_data = None
target_found = False
detected_aruco_ids = []
frame_count = 0   # DEBUG: kac frame geldi sayaci

qr_detector = cv2.QRCodeDetector()
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
aruco_params = cv2.aruco.DetectorParameters()
aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)


def on_lidar(msg):
    global latest_ranges
    latest_ranges = list(msg.ranges)


def on_image(msg):
    global qr_detected_data, detected_aruco_ids, frame_count
    try:
        frame_count += 1
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        frame = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        data, points, _ = qr_detector.detectAndDecode(frame)
        if data:
            qr_detected_data = data

        corners, ids, _ = aruco_detector.detectMarkers(gray)
        if ids is not None:
            detected_aruco_ids = ids.flatten().tolist()
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        else:
            detected_aruco_ids = []

        # DEBUG: her 10 frame'de bir diske kaydet (surekli yazmak yavaslatmasin)
        if frame_count % 10 == 0:
            cv2.imwrite("/home/furk/autonomous_landing/debug_warehouse_cam.png", frame)
    except Exception as e:
        send_intel(f"KAMERA HATASI: {e}")


async def get_altitude(drone):
    global current_altitude
    try:
        async for pos in drone.telemetry.position():
            current_altitude = pos.relative_altitude_m
    except Exception:
        pass


async def get_health(drone):
    global sensors_ready
    try:
        async for h in drone.telemetry.health():
            sensors_ready = (
                h.is_global_position_ok
                and h.is_local_position_ok
                and h.is_home_position_ok
            )
    except Exception:
        pass


def get_distances():
    if latest_ranges is None:
        return 99.0, 99.0, 99.0
    n = len(latest_ranges)
    front = latest_ranges[n // 2]
    left = latest_ranges[n // 4]
    right = latest_ranges[3 * n // 4]
    front = front if 0.05 < front < 30 else 30.0
    left = left if 0.05 < left < 30 else 30.0
    right = right if 0.05 < right < 30 else 30.0
    return front, left, right


async def check_qr():
    global qr_detected_data, target_found
    if qr_detected_data and not target_found:
        send_intel(f"QR OKUNDU: {qr_detected_data}")
        if qr_detected_data == TARGET_QR:
            send_intel(f"★ HEDEF BULUNDU: {TARGET_QR} ★")
            target_found = True
            return True
        qr_detected_data = None
    return False


def check_corridor_end(expected_id):
    return expected_id in detected_aruco_ids


async def emergency_check(drone):
    front, left, right = get_distances()
    if left < EMERGENCY_SIDE_DIST:
        send_intel(f"SOLDA ACIL! ({left:.1f}m) Kaciliyor...")
        for _ in range(5):
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.4, 0.0, 0.0))
            await asyncio.sleep(0.1)
        return True
    if right < EMERGENCY_SIDE_DIST:
        send_intel(f"SAGDA ACIL! ({right:.1f}m) Kaciliyor...")
        for _ in range(5):
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, -0.4, 0.0, 0.0))
            await asyncio.sleep(0.1)
        return True
    return False


async def turn_90(drone):
    send_intel("90 derece donuluyor...")
    for _ in range(10):
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(0.1)

    turn_duration = 90.0 / TURN_SPEED
    step = 0.1
    steps = int(turn_duration / step)
    for _ in range(steps):
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, TURN_SPEED))
        await asyncio.sleep(step)

    for _ in range(8):
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(0.1)


async def fly_forward_pass(drone, expected_end_id):
    elapsed = 0.0
    step = 0.1

    while elapsed < MAX_PASS_DURATION:
        if await check_qr():
            return "target_found"

        if check_corridor_end(expected_end_id):
            send_intel(f"Koridor sonu markeri (ID:{expected_end_id}) goruldu!")
            break

        if await emergency_check(drone):
            elapsed += 0.5
            continue

        front, left, right = get_distances()
        if front < FALLBACK_FRONT_DIST:
            send_intel(f"YEDEK GUVENLIK: onde engel ({front:.1f}m). Duruluyor.")
            break

        # KORIDOR ORTALAMA: sol/sag mesafe farkina gore hafif yan duzeltme.
        # Donus kararini HALA marker veriyor, bu sadece duz gitmeyi sagliyor.
        CENTERING_KP = 0.15
        MAX_CENTER_CORRECTION = 0.3
        side_diff = right - left   # sagda daha fazla bosluk varsa pozitif -> saga kay
        side_diff_clamped = min(max(side_diff, -3.0), 3.0)
        vy_correction = 0.0
        if left < 5.0 and right < 5.0:  # sadece gercekten koridordaysak duzelt
            vy_correction = max(min(side_diff_clamped * CENTERING_KP, MAX_CENTER_CORRECTION), -MAX_CENTER_CORRECTION)

        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(FORWARD_SPEED, vy_correction, 0.0, 0.0))
        await asyncio.sleep(step)
        elapsed += step

        # DEBUG: her 5 saniyede bir goruntude ne var raporla
        if int(elapsed * 10) % 50 == 0:
            send_intel(f"DEBUG t={elapsed:.1f}s gorulen ArUco ID'ler: {detected_aruco_ids}")

    for _ in range(5):
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(0.1)
    return "corridor_done"


async def shift_to_next_corridor(drone, duration):
    send_intel("Sonraki koridora geciliyor...")
    elapsed = 0.0
    step = 0.1
    while elapsed < duration:
        if await emergency_check(drone):
            elapsed += 0.5
            continue
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(STRAFE_SPEED, 0.0, 0.0, 0.0))
        await asyncio.sleep(step)
        elapsed += step
    for _ in range(5):
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(0.1)


async def run():
    send_intel(f"Depo taramasi basliyor. Hedef: {TARGET_QR}")
    send_intel(f"Marker plani: {END_MARKER_IDS}")

    lidar_node = trans.Node()
    lidar_node.subscribe(LaserScan, LIDAR_TOPIC, on_lidar)
    cam_node = trans.Node()
    cam_node.subscribe(Image, CAMERA_TOPIC, on_image)

    drone = System()
    await drone.connect(system_address="udp://:14540")
    send_intel("Drone baglantisi kuruluyor...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            break

    asyncio.create_task(get_altitude(drone))
    asyncio.create_task(get_health(drone))

    send_intel("Sensorler bekleniyor...")
    while not sensors_ready:
        await asyncio.sleep(0.2)

    send_intel("Kalkis basliyor...")
    await drone.action.arm()
    await drone.action.takeoff()
    while current_altitude < 1.4:
        await asyncio.sleep(0.1)

    send_intel(f"Irtifa {current_altitude:.1f}m! Marker-tetiklemeli tarama basliyor...")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    try:
        await drone.offboard.start()
    except OffboardError as e:
        send_intel(f"Offboard hatasi: {e}")
        return

    found = False
    for corridor_i in range(NUM_CORRIDORS):
        expected_id = END_MARKER_IDS[corridor_i]
        send_intel(f"--- Koridor {corridor_i+1}/{NUM_CORRIDORS} taraniyor (bitis markeri: ID{expected_id}) ---")

        result = await fly_forward_pass(drone, expected_id)
        if result == "target_found":
            found = True
            break

        if corridor_i < NUM_CORRIDORS - 1:
            await turn_90(drone)
            await shift_to_next_corridor(drone, SIDE_SHIFT_DURATION)
            await turn_90(drone)

    if found:
        send_intel(f"HEDEF BULUNDU: {TARGET_QR}! Iniliyor...")
    else:
        send_intel("Tum koridorlar tarandi, hedef bu katta bulunamadi. Iniliyor...")

    await drone.offboard.stop()
    await drone.action.land()
    send_intel("GOREV TAMAMLANDI!")


if __name__ == "__main__":
    asyncio.run(run())
