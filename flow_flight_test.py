"""
Basic flight test: can the vehicle take off and hold position using optical
flow alone, with GPS disabled? This is the foundation everything else builds on.
"""
import os
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"

import asyncio
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed, OffboardError


async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("[INFO] Connecting")
    async for state in drone.core.connection_state():
        if state.is_connected:
            break
    print("[INFO] Connected")

    # Watch health flags, especially local_position_ok which depends on flow
    print("[INFO] Waiting for sensor health (optical flow and EKF2 must settle)")
    sensors_ready = False
    timeout_counter = 0
    async for h in drone.telemetry.health():
        print(f"  gyro={h.is_gyrometer_calibration_ok}  accel={h.is_accelerometer_calibration_ok}  "
              f"mag={h.is_magnetometer_calibration_ok}  local_pos={h.is_local_position_ok}  "
              f"home={h.is_home_position_ok}  global_pos={h.is_global_position_ok}")
        if h.is_local_position_ok and h.is_home_position_ok:
            sensors_ready = True
            break
        timeout_counter += 1
        if timeout_counter > 60:  # roughly 30 s at ~0.5 s per health update
            print("[WARN] Sensors not ready after 30 s, continuing anyway")
            break

    print(f"\n[INFO] Sensor status: {'READY' if sensors_ready else 'NOT READY (proceeding anyway)'}")

    print("[INFO] Arming")
    try:
        await drone.action.arm()
    except Exception as e:
        print(f"[ERROR] Arming failed: {e}")
        return

    print("[INFO] Taking off")
    await drone.action.takeoff()

    print("[INFO] Monitoring altitude and velocity for 15 s")
    for i in range(15):
        async for pos in drone.telemetry.position():
            alt = pos.relative_altitude_m
            break
        async for odom in drone.telemetry.position_velocity_ned():
            vx = odom.velocity.north_m_s
            vy = odom.velocity.east_m_s
            break
        print(f"  t={i}s  altitude={alt:.2f} m  v_north={vx:.2f}  v_east={vy:.2f}")
        await asyncio.sleep(1)

    print("\n[INFO] Entering offboard mode for position-hold test")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"[ERROR] Offboard rejected: {e}")
        await drone.action.land()
        return

    print("[INFO] Holding position for 10 s to measure drift")
    positions = []
    for i in range(20):
        async for odom in drone.telemetry.position_velocity_ned():
            positions.append((odom.position.north_m, odom.position.east_m))
            break
        await asyncio.sleep(0.5)

    if positions:
        start_n, start_e = positions[0]
        end_n, end_e = positions[-1]
        drift = ((end_n - start_n)**2 + (end_e - start_e)**2) ** 0.5
        print(f"\n[RESULT] Total drift over 10 s: {drift:.3f} m")
        print(f"  Start: N={start_n:.2f} E={start_e:.2f}")
        print(f"  End:   N={end_n:.2f} E={end_e:.2f}")

    print("\n[INFO] Test complete, landing")
    await drone.offboard.stop()
    await drone.action.land()


if __name__ == "__main__":
    asyncio.run(run())
