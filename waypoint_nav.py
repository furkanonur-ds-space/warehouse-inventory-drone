"""
Waypoint navigation test. No camera processing, motion only.

Because the warehouse floor plan is known in advance, the route is fixed
rather than discovered during flight.

Route pattern (boustrophedon / zigzag):
  Corridor 1, level 1: south to north
  Corridor 1, level 2: north to south
  Corridor 1, level 3: south to north
  Corridor 2, level 3: north to south
  ...
"""
import os
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"

import asyncio
import math
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError

# --- WAREHOUSE FLOOR PLAN (known in advance) ---
CORRIDOR_X = [-8.5, -3.9, 0.7, 6.9]     # aisle centre lines
FLIGHT_Z = [0.5, 1.15, 1.8]              # flight altitudes, just above each shelf
Y_SOUTH, Y_NORTH = -8.0, 8.0             # aisle end points, clear of the walls

# Only the first two aisles are flown, to keep the test short
CORRIDORS_TO_SCAN = 2

WAYPOINT_TOLERANCE = 0.4   # metres, waypoint counts as reached below this
TIMEOUT_PER_WP = 40         # seconds, maximum time allowed per waypoint


def build_route():
    """Build the boustrophedon route as a list of (x, y, z, yaw) tuples."""
    route = []
    going_north = True

    for c_idx in range(CORRIDORS_TO_SCAN):
        x = CORRIDOR_X[c_idx]
        # Level order alternates so no empty return leg is needed
        levels = FLIGHT_Z if c_idx % 2 == 0 else list(reversed(FLIGHT_Z))

        for z in levels:
            if going_north:
                route.append((x, Y_SOUTH, z, 0.0))   # start of the aisle
                route.append((x, Y_NORTH, z, 0.0))   # far end of the aisle
            else:
                route.append((x, Y_NORTH, z, 0.0))
                route.append((x, Y_SOUTH, z, 0.0))
            going_north = not going_north

    return route


# --- STATE ---
current_pos = {"n": 0.0, "e": 0.0, "d": 0.0}


async def track_position(drone):
    global current_pos
    try:
        async for odom in drone.telemetry.position_velocity_ned():
            current_pos["n"] = odom.position.north_m
            current_pos["e"] = odom.position.east_m
            current_pos["d"] = odom.position.down_m
    except Exception:
        pass


def distance_to(target_n, target_e, target_d):
    dn = target_n - current_pos["n"]
    de = target_e - current_pos["e"]
    dd = target_d - current_pos["d"]
    return math.sqrt(dn*dn + de*de + dd*dd)


async def goto_waypoint(drone, wp_idx, total, x, y, z, yaw):
    """
    Fly to a waypoint.

    PX4 works in NED, which is related to the Gazebo world frame by an axis
    swap, an origin shift to the spawn point, and a sign flip on the vertical
    axis. All three conversions happen below.
    """
    # Convert Gazebo world coordinates into NED relative to the spawn point
    SPAWN_X, SPAWN_Y = -8.5, -9.0

    # The vehicle spawns with 90 degrees of yaw, so Gazebo X maps to NED East
    # and Gazebo Y maps to NED North
    target_n = y - SPAWN_Y
    target_e = x - SPAWN_X
    target_d = -z    # NED counts down as positive, so altitude is negated

    print(f"\n[WP {wp_idx}/{total}] target Gazebo(x={x:.1f}, y={y:.1f}, z={z:.2f})")
    print(f"           NED equivalent: N={target_n:.2f} E={target_e:.2f} D={target_d:.2f}")

    elapsed = 0.0
    step = 0.2
    while elapsed < TIMEOUT_PER_WP:
        await drone.offboard.set_position_ned(
            PositionNedYaw(target_n, target_e, target_d, yaw))

        dist = distance_to(target_n, target_e, target_d)
        if dist < WAYPOINT_TOLERANCE:
            print(f"           reached (error {dist:.2f} m, {elapsed:.1f} s)")
            return True

        if int(elapsed * 5) % 25 == 0:  # status line roughly every 5 s
            print(f"           distance {dist:.2f} m  position N={current_pos['n']:.2f} E={current_pos['e']:.2f} D={current_pos['d']:.2f}")

        await asyncio.sleep(step)
        elapsed += step

    print(f"           timeout, remaining {distance_to(target_n, target_e, target_d):.2f} m")
    return False


async def run():
    route = build_route()
    print("="*60)
    print("  WAYPOINT NAVIGATION TEST")
    print(f"  Waypoints:        {len(route)}")
    print(f"  Corridors:        {CORRIDORS_TO_SCAN}")
    print("="*60)

    drone = System()
    await drone.connect(system_address="udp://:14540")
    print("\n[INFO] Connecting")
    async for state in drone.core.connection_state():
        if state.is_connected:
            break

    asyncio.create_task(track_position(drone))

    print("[INFO] Waiting for position estimate")
    async for h in drone.telemetry.health():
        if h.is_local_position_ok and h.is_home_position_ok:
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
    except OffboardError as e:
        print(f"[ERROR] Offboard rejected: {e}")
        await drone.action.land()
        return

    print("\n[INFO] Starting route\n")
    success_count = 0
    for i, (x, y, z, yaw) in enumerate(route, 1):
        ok = await goto_waypoint(drone, i, len(route), x, y, z, yaw)
        if ok:
            success_count += 1

    print("\n" + "="*60)
    print("  ROUTE COMPLETE")
    print(f"  Waypoints reached: {success_count}/{len(route)}")
    print("="*60)

    print("\n[INFO] Landing")
    await drone.offboard.stop()
    await drone.action.land()


if __name__ == "__main__":
    asyncio.run(run())
