import os
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"

import asyncio
import cv2
import numpy as np
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed
import gz.transport13 as trans
from gz.msgs10.image_pb2 import Image

latest_frame = None
target_found = False
err_x = 0  
err_y = 0  
current_altitude = 0.0
sensors_ready = False

def on_image(msg):
    global latest_frame, target_found, err_x, err_y
    try:
        img = np.frombuffer(msg.data, dtype=np.uint8)
        img = img.reshape((msg.height, msg.width, 3))
        frame = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        frame_h, frame_w = frame.shape[:2]
        center_x, center_y = frame_w // 2, frame_h // 2

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        found = False
        for cnt in contours:
            if cv2.contourArea(cnt) > 5000:
                peri = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)
                if len(approx) == 4:
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        err_x = cx - center_x
                        err_y = cy - center_y
                        
                        cv2.drawContours(frame, [approx], -1, (0, 255, 0), 3)
                        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                        cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)
                        cv2.line(frame, (center_x, center_y), (cx, cy), (0, 255, 255), 2)
                        found = True
                        break
        target_found = found
        latest_frame = frame
    except Exception as e:
        pass

async def smart_sleep(seconds):
    for _ in range(int(seconds * 10)):
        if latest_frame is not None:
            cv2.imshow("Autonomous Landing Camera", latest_frame)
            cv2.waitKey(1)
        await asyncio.sleep(0.1)

# Background tasks guarded by an exception handler
async def get_altitude(drone):
    global current_altitude
    try:
        async for pos in drone.telemetry.position():
            current_altitude = pos.relative_altitude_m
    except:
        pass

async def get_health(drone):
    global sensors_ready
    try:
        async for health in drone.telemetry.health():
            if health.is_global_position_ok and health.is_local_position_ok and health.is_home_position_ok:
                sensors_ready = True
            else:
                sensors_ready = False
    except:
        pass

async def run():
    node = trans.Node()
    topic = "/world/default/model/x500_0/link/base_link/sensor/camera/image"
    node.subscribe(Image, topic, on_image)

    drone = System()
    await drone.connect(system_address="udp://:14540")
    
    print("[INFO] Connecting to vehicle...")
    async for state in drone.core.connection_state():
        if state.is_connected: break

    asyncio.create_task(get_altitude(drone))
    asyncio.create_task(get_health(drone))

    print("[INFO] Waiting for sensors...")
    while not sensors_ready:
        if latest_frame is not None:
            cv2.imshow("Autonomous Landing Camera", latest_frame)
            cv2.waitKey(1)
        await asyncio.sleep(0.1)

    print("[INFO] Arming motors...")
    await drone.action.arm()
    await drone.action.takeoff()

    print("[INFO] Waiting for the vehicle to leave the ground...")
    while current_altitude < 2.0:
        if latest_frame is not None:
            cv2.imshow("Autonomous Landing Camera", latest_frame)
            cv2.waitKey(1)
        await asyncio.sleep(0.1)

    # No fixed wait here: go straight to offboard once airborne.
    print(f"[SUCCESS] Altitude {current_altitude:.2f} m. Switching to offboard...")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    await drone.offboard.start()

    Kp = 0.003 

    while True:
        if latest_frame is not None:
            cv2.imshow("Autonomous Landing Camera", latest_frame)
            cv2.waitKey(1)

        if target_found:
            vx = max(min(-err_y * Kp, 0.3), -0.3)
            vy = max(min(err_x * Kp, 0.3), -0.3)

            if abs(err_x) < 30 and abs(err_y) < 30:
                vz = 0.4  
                if current_altitude < 0.35:
                    print("[SUCCESS] Landed on target. Disarming...")
                    await drone.offboard.stop()
                    await drone.action.land()
                    print("[INFO] Landing camera recording for another 8 seconds...")
                    await smart_sleep(8) 
                    break
            else:
                vz = 0.0  
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(vx, vy, vz, 0.0))
        else:
            if 0.1 < current_altitude < 0.6:
                print("[INFO] Target left the frame. Continuing blind descent.")
                await drone.offboard.stop()
                await drone.action.land()
                print("[INFO] Watching the descent (8 s)...")
                await smart_sleep(8) 
                print("[SUCCESS] Mission complete.")
                break
            else:
                await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

        await asyncio.sleep(0.05)

if __name__ == "__main__":
    asyncio.run(run())
