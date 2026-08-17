import os
import sys
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"

import asyncio
import cv2
import numpy as np
import socket
import requests
import time
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed, OffboardError
import gz.transport13 as trans
from gz.msgs10.image_pb2 import Image

# --- DRONE KIMLIK AYARLARI ---
drone_idx   = int(sys.argv[1]) if len(sys.argv) > 1 else 0
DRONE_ID    = f"DRONE-{drone_idx + 1}"
MAVSDK_PORT = 14540 + drone_idx
GRPC_PORT   = 50051 + drone_idx
CAMERA_TOPIC = f"/world/default/model/x500_{drone_idx}/link/base_link/sensor/camera/image"

# --- SUNUCU AYARLARI ---
SERVER_URL = "http://127.0.0.1:5000/detect"
last_send_time = 0

def send_to_server(x, y):
    payload = {"object": f"Kirmizi Kutu ({DRONE_ID})", "coordinates": {"x": x, "y": y}}
    try:
        response = requests.post(SERVER_URL, json=payload, timeout=1.0)
        if response.status_code == 200:
            print(f"[{DRONE_ID}] Kirmizi kutu basariyla sunucuya iletildi!")
    except Exception as e:
        # Sessizce gecmek yerine hatayi basalim ki neden gitmedigini gorelim
        print(f"[{DRONE_ID}] Sunucu iletim hatasi: {e}")

# --- UDP ISTIHBARAT ---
udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
def send_intel(msg):
    full = f"[{DRONE_ID}] {msg}"
    udp_sock.sendto(full.encode("utf-8"), ("127.0.0.1", 5005))
    print(full)

# --- DURUM DEGISKENLERI ---
latest_frame         = None
current_altitude     = 0.0
sensors_ready        = False
red_box_found        = False
landing_target_found = False
landing_err_x        = 0
landing_err_y        = 0
frame_counter        = 0

# --- KAMERA CALLBACK ---
def on_image(msg):
    global latest_frame, red_box_found, landing_target_found, landing_err_x, landing_err_y, frame_counter, last_send_time
    
    frame_counter += 1
    if frame_counter % 2 != 0: return 

    try:
        img   = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        frame = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        fh, fw = frame.shape[:2]
        cx_f, cy_f = fw // 2, fh // 2

        # -- KIRMIZI KUTU TESPITI VE SUNUCU BILDIRIMI --
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        m1  = cv2.inRange(hsv, np.array([0,   120, 70]), np.array([10,  255, 255]))
        m2  = cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))
        cnts_red, _ = cv2.findContours(m1 + m2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        red_box_found = False
        if cnts_red:
            largest = max(cnts_red, key=cv2.contourArea)
            if cv2.contourArea(largest) > 500:
                M = cv2.moments(largest)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    x, y, w, h = cv2.boundingRect(largest)
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
                    
                    red_box_found = True

                    current_time = time.time()
                    if current_time - last_send_time > 1.0:
                        send_to_server(cx, cy)
                        last_send_time = current_time

        # -- INIS HEDEFI (ARUCO KAREKOD TESPITI) --
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100) # Gazebo genelde 5x5 kullanir
        aruco_params = cv2.aruco.DetectorParameters()
        
        # OpenCV surumune gore ArUco tespiti
        try:
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        except AttributeError:
            # OpenCV 4.7 ve uzeri icin yeni yontem
            detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
            corners, ids, rejected = detector.detectMarkers(gray)

        landing_target_found = False
        if ids is not None and len(ids) > 0:
            # Bulunan ilk marker'in koselerini al ve merkezini hesapla
            c = corners[0][0]
            M_cx = int(np.mean(c[:, 0]))
            M_cy = int(np.mean(c[:, 1]))
            
            landing_err_x = M_cx - cx_f
            landing_err_y = M_cy - cy_f
            landing_target_found = True
            
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.circle(frame, (M_cx, M_cy), 5, (0, 0, 255), -1)

        latest_frame = frame
    except Exception as e:
        pass

# --- ARKA PLAN GOREVLERI ---
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

async def show_camera():
    while True:
        if latest_frame is not None:
            cv2.imshow(f"Kamera - {DRONE_ID}", latest_frame)
            cv2.waitKey(1)
        await asyncio.sleep(0.05)

async def move_offboard(drone, vx, vy, vz, vyaw, duration):
    steps = max(1, int(duration * 10))
    for _ in range(steps):
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(vx, vy, vz, vyaw))
        await asyncio.sleep(0.1)

# --- ANA GOREV ---
async def run(ready_event: asyncio.Event, start_event: asyncio.Event, takeoff_ready_event: asyncio.Event, takeoff_start_event: asyncio.Event):
    node = trans.Node()
    node.subscribe(Image, CAMERA_TOPIC, on_image)

    drone = System(port=GRPC_PORT)
    await drone.connect(system_address=f"udpin://127.0.0.1:{MAVSDK_PORT}")

    send_intel("Baglanti kuruluyor...")
    async for state in drone.core.connection_state():
        if state.is_connected: break

    asyncio.create_task(get_altitude(drone))
    asyncio.create_task(get_health(drone))
    asyncio.create_task(show_camera()) 

    send_intel("Sensorler bekleniyor...")
    while not sensors_ready:
        await asyncio.sleep(0.1)

    send_intel("Hazir! Yerde senkronizasyon bekleniyor...")
    ready_event.set()          
    await start_event.wait()   
    send_intel("SENKRONIZE KALKIS!")

    await drone.action.arm()
    await drone.action.takeoff()
    
    timeout_counter = 0
    while current_altitude < 1.8 and timeout_counter < 150:
        await asyncio.sleep(0.1)
        timeout_counter += 1

    send_intel(f"Irtifaya ({current_altitude:.2f}m) ulasildi. Havada diger drone bekleniyor...")
    takeoff_ready_event.set()
    await takeoff_start_event.wait()
    send_intel("HAVADA SENKRONIZASYON SAGLANDI! Offboard moda geciliyor...")

    offboard_started = False
    for _ in range(5):
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        try:
            await drone.offboard.start()
            offboard_started = True
            break
        except OffboardError as e:
            send_intel(f"Offboard reddedildi (Tekrar deneniyor): {e}")
            await asyncio.sleep(0.5)

    if not offboard_started:
        send_intel("KRITIK HATA: Offboard moda gecilemedi, drone oldugu yerde asili kalacak!")
        return

    # --- KARE DEVRİYE ---
    send_intel("Kare devriye basliyor...")
    red_reported = False
    for i in range(4):
        send_intel(f"{i+1}. kenar: ileri...")
        await move_offboard(drone, 1.5, 0.0, 0.0, 0.0, 4.0)

        if red_box_found and not red_reported:
            send_intel("KIRMIZI KUTU TESPIT EDILDI! Sunucuya bildiriliyor...")
            red_reported = True

        send_intel(f"{i+1}. kose: donuluyor...")
        await move_offboard(drone, 0.0, 0.0, 0.0, 30.0, 3.0)
        await move_offboard(drone, 0.0, 0.0, 0.0, 0.0, 1.0)

    send_intel("Devriye tamamlandi! Hassas inis hedefi araniyor...")

    # --- HASSAS INIS ---
    Kp = 0.003
    marker_reported = False
    while True:
        if landing_target_found:
            if not marker_reported:
                send_intel("ARUCO INIS HEDEFI KILITLENDI!")
                marker_reported = True
            vx = max(min(-landing_err_y * Kp, 0.3), -0.3)
            vy = max(min( landing_err_x * Kp, 0.3), -0.3)
            if abs(landing_err_x) < 30 and abs(landing_err_y) < 30:
                vz = 0.4
                if current_altitude < 0.35:
                    send_intel("HASSAS INIS TAMAMLANDI!")
                    await drone.offboard.stop()
                    await drone.action.land()
                    break
            else:
                vz = 0.0
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(vx, vy, vz, 0.0))
        else:
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.3, 0.0, 0.0, 0.0))
            if 0.1 < current_altitude < 0.6 and marker_reported:
                send_intel("Kor inis yapiliyor...")
                await drone.offboard.stop()
                await drone.action.land()
                break
        await asyncio.sleep(0.05)

    send_intel("GOREV TAMAMLANDI!")
