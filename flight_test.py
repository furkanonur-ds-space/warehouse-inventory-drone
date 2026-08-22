import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("[INFO] Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("[SUCCESS] Drone connection established!")
            break

    print("[INFO] Waiting for GPS/Sensors to warm up...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_local_position_ok:
            print("[SUCCESS] Sensors ready! We are good to go.")
            break
        await asyncio.sleep(1)

    print("[INFO] Arming motors...")
    await drone.action.arm()

    print("[INFO] Taking off (This might be in slow-motion due to WSL, grab a coffee!)...")
    await drone.action.takeoff()
    
    # Live altitude tracking, so the climb can be watched as it happens.
    async for position in drone.telemetry.position():
        altitude = position.relative_altitude_m
        print(f"Current Altitude: {altitude:.2f} meters")
        if altitude >= 2.0:
            print("[SUCCESS] Target altitude reached safely!")
            break

    # Give the gRPC channel two seconds to recover after breaking out of
    # the telemetry loop. Without this the connection crashes.
    await asyncio.sleep(2)

    print("[INFO] Setting initial setpoint...")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -2.5, 0.0))
    
    print("[INFO] Starting Offboard mode...")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"[ERROR] Starting offboard mode failed: {error}")
        return

    print("[INFO] Flying 2 meters forward...")
    await drone.offboard.set_position_ned(PositionNedYaw(2.0, 0.0, -2.5, 0.0))
    
    # Allow plenty of time for the forward leg, since SITL runs slowly (20 s)
    print("[INFO] Waiting for drone to reach the target...")
    await asyncio.sleep(20)

    print("[INFO] Mission completed, landing...")
    await drone.offboard.stop()
    await drone.action.land()

if __name__ == "__main__":
    asyncio.run(run())
