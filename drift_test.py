"""
TANI TESTI: Drone sadece duz ileri ucar (hicbir duzeltme yok).
Gercek Gazebo konumunu (ground truth) her 0.5 saniyede kaydeder.
Amac: kaymanin gercekten oldugunu ve ne kadar oldugunu OLCMEK.
"""
import os
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"

import asyncio
import subprocess
import re
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed, OffboardError


def get_ground_truth_pose():
    """gz topic ile gercek dunya konumunu okur"""
    try:
        result = subprocess.run(
            ["gz", "topic", "-e", "-t", "/world/warehouse/pose/info", "-n", "1"],
            capture_output=True, text=True, timeout=3
        )
        text = result.stdout
        # x500_lidar_2d_0 modelinin pose blogunu bul
        idx = text.find('name: "x500_lidar_2d_0"')
        if idx == -1:
            return None
        chunk = text[idx:idx+400]
        x = re.search(r'x:\s*([-\d.e]+)', chunk)
        y = re.search(r'y:\s*([-\d.e]+)', chunk)
        z_orient = re.search(r'orientation\s*\{[^}]*z:\s*([-\d.e]+)', chunk)
        w_orient = re.search(r'w:\s*([-\d.e]+)', chunk)
        if x and y:
            return float(x.group(1)), float(y.group(1))
    except Exception as e:
        print(f"Pose okuma hatasi: {e}")
    return None


async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")
    print("Baglaniliyor...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            break

    print("Arm ve takeoff...")
    await drone.action.arm()
    await drone.action.takeoff()
    await asyncio.sleep(8)

    print("Offboard baslatiliyor...")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"Offboard hatasi: {e}")
        return

    print("\n--- SADECE ILERI UCUS BASLIYOR (hicbir duzeltme yok) ---")
    print("t(s)  |  X konum  |  Y konum  |  Y_kayma (baslangica gore)")
    print("-" * 60)

    initial_pose = get_ground_truth_pose()
    if initial_pose is None:
        print("UYARI: Ground truth pose okunamadi, gz topic calismiyor olabilir")
        initial_y = -4.0
    else:
        initial_y = initial_pose[1]

    for i in range(20):  # 10 saniye, 0.5s araliklarla
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.5, 0.0, 0.0, 0.0))
        await asyncio.sleep(0.5)

        pose = get_ground_truth_pose()
        if pose:
            x, y = pose
            drift = y - initial_y
            print(f"{i*0.5:4.1f}  |  {x:7.2f}  |  {y:7.2f}  |  {drift:+.3f}")
        else:
            print(f"{i*0.5:4.1f}  |  OKUNAMADI")

    print("\n--- TEST BITTI ---")
    await drone.offboard.stop()
    await drone.action.land()


if __name__ == "__main__":
    asyncio.run(run())
