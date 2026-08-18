"""
Day 0 baseline: how far does the position estimate drift over a full mission?

Flies the complete 24-waypoint route while logging, every 0.5 s, both the
estimate PX4 believes and the true pose reported by Gazebo. The difference
between them is the localization error.

Ground truth is read ONLY to measure error. It is never fed back into the
estimator or used for control. Mixing the two would make the measurement
meaningless.

Output: drift_baseline.json with per-sample error, median and P95.

Usage:
    python3 drift_profile.py            # full route
    python3 drift_profile.py --hover 60 # stationary hover instead
"""
import os
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"

import argparse
import asyncio
import json
import math
import re
import statistics
import time
from datetime import datetime

from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError
import gz.transport13 as trans
from gz.msgs10.pose_v_pb2 import Pose_V

# --- WAREHOUSE FLOOR PLAN (must match warehouse_scanner.py) ---
CORRIDOR_X = [-8.5, -3.9, 0.7, 5.3]
FLIGHT_Z = [0.5, 1.15, 1.8]
Y_SOUTH, Y_NORTH = -8.0, 8.0
SPAWN_X, SPAWN_Y = -8.5, -9.0

CRUISE_SPEED = 0.7
WAYPOINT_TOLERANCE = 0.4
TIMEOUT_MARGIN = 15.0
SAMPLE_PERIOD = 0.5

WORLD = "warehouse_v2"
DRONE_MODEL = "x500_scanner_0"
OUTPUT_JSON = os.path.expanduser("~/autonomous_landing/drift_baseline.json")

current_pos = {"n": 0.0, "e": 0.0, "d": 0.0}
samples = []
_sampling = True


# Ground truth is received passively from Gazebo's pose topic.
#
# An earlier version shelled out to `gz model -m ... -p` on every sample.
# Spawning a process twice a second competes with the simulator for CPU and
# noticeably slowed the whole system down. Subscribing to the pose stream
# costs nothing per sample.
_truth = {"x": None, "y": None, "z": None}


def on_pose_info(msg):
    """Extract this vehicle's pose from the world pose stream."""
    try:
        for pose in msg.pose:
            if pose.name == DRONE_MODEL:
                _truth["x"] = pose.position.x
                _truth["y"] = pose.position.y
                _truth["z"] = pose.position.z
                return
    except Exception:
        pass


def read_ground_truth():
    if _truth["x"] is None:
        return None
    return _truth["x"], _truth["y"], _truth["z"]


def ned_to_gazebo(n, e, d):
    return e + SPAWN_X, n + SPAWN_Y, -d


async def track_position(drone):
    global current_pos
    try:
        async for odom in drone.telemetry.position_velocity_ned():
            current_pos["n"] = odom.position.north_m
            current_pos["e"] = odom.position.east_m
            current_pos["d"] = odom.position.down_m
    except Exception:
        pass


async def sampler(start_time):
    """Log estimate against ground truth at a fixed rate for the whole flight."""
    while _sampling:
        truth = read_ground_truth()
        if truth is not None:
            est_x, est_y, est_z = ned_to_gazebo(
                current_pos["n"], current_pos["e"], current_pos["d"])
            true_x, true_y, true_z = truth

            err_h = math.sqrt((est_x - true_x) ** 2 + (est_y - true_y) ** 2)
            err_v = abs(est_z - true_z)
            err_3d = math.sqrt(err_h ** 2 + err_v ** 2)

            samples.append({
                "t": round(time.time() - start_time, 2),
                "estimate": [round(est_x, 3), round(est_y, 3), round(est_z, 3)],
                "truth": [round(true_x, 3), round(true_y, 3), round(true_z, 3)],
                "error_horizontal": round(err_h, 4),
                "error_vertical": round(err_v, 4),
                "error_3d": round(err_3d, 4),
            })
        await asyncio.sleep(SAMPLE_PERIOD)


def build_route():
    route = []
    heading_north = True
    for corridor_index, x in enumerate(CORRIDOR_X):
        levels = FLIGHT_Z if corridor_index % 2 == 0 else list(reversed(FLIGHT_Z))
        for z in levels:
            if heading_north:
                route.append((x, Y_SOUTH, z))
                route.append((x, Y_NORTH, z))
            else:
                route.append((x, Y_NORTH, z))
                route.append((x, Y_SOUTH, z))
            heading_north = not heading_north
    return route


def distance_to(tn, te, td):
    dn = tn - current_pos["n"]
    de = te - current_pos["e"]
    dd = td - current_pos["d"]
    return math.sqrt(dn * dn + de * de + dd * dd)


async def goto_waypoint(drone, index, total, x, y, z):
    """Carrot following, identical to warehouse_scanner.py."""
    target_n = y - SPAWN_Y
    target_e = x - SPAWN_X
    target_d = -z

    start_n, start_e, start_d = (current_pos["n"], current_pos["e"],
                                 current_pos["d"])
    leg_length = math.sqrt((target_n - start_n) ** 2 +
                           (target_e - start_e) ** 2 +
                           (target_d - start_d) ** 2)

    dt = 0.1
    travelled = 0.0
    elapsed = 0.0
    max_time = leg_length / CRUISE_SPEED + TIMEOUT_MARGIN

    while elapsed < max_time:
        travelled = min(leg_length, travelled + CRUISE_SPEED * dt)
        fraction = 1.0 if leg_length == 0 else travelled / leg_length
        await drone.offboard.set_position_ned(PositionNedYaw(
            start_n + (target_n - start_n) * fraction,
            start_e + (target_e - start_e) * fraction,
            start_d + (target_d - start_d) * fraction,
            0.0))

        if (travelled >= leg_length
                and distance_to(target_n, target_e, target_d) < WAYPOINT_TOLERANCE):
            err = samples[-1]["error_3d"] if samples else float("nan")
            print(f"[{index}/{total}] reached x={x:.1f} y={y:.1f} z={z:.2f}"
                  f"   current error {err:.3f} m")
            return True

        await asyncio.sleep(dt)
        elapsed += dt

    print(f"[{index}/{total}] TIMEOUT, remaining "
          f"{distance_to(target_n, target_e, target_d):.2f} m")
    return False


def summarize(mode, reached, total, duration):
    if not samples:
        print("\n[ERROR] No samples collected. Is Gazebo running and is the "
              "model name correct?")
        return

    h_errors = [s["error_horizontal"] for s in samples]
    v_errors = [s["error_vertical"] for s in samples]
    e_errors = [s["error_3d"] for s in samples]

    def p95(values):
        ordered = sorted(values)
        return ordered[min(len(ordered) - 1, int(len(ordered) * 0.95))]

    median_h = statistics.median(h_errors)
    p95_h = p95(h_errors)

    print("\n" + "=" * 62)
    print("  DRIFT BASELINE")
    print(f"  Mode              : {mode}")
    print(f"  Duration          : {duration:.0f} s")
    print(f"  Samples           : {len(samples)}")
    if total:
        print(f"  Waypoints reached : {reached}/{total}")
    print("-" * 62)
    print(f"  Horizontal error  median {median_h:.3f} m   "
          f"P95 {p95_h:.3f} m   max {max(h_errors):.3f} m")
    print(f"  Vertical error    median {statistics.median(v_errors):.3f} m   "
          f"P95 {p95(v_errors):.3f} m   max {max(v_errors):.3f} m")
    print(f"  3D error          median {statistics.median(e_errors):.3f} m   "
          f"P95 {p95(e_errors):.3f} m   max {max(e_errors):.3f} m")
    print("-" * 62)
    print("  Sprint target A3  median <= 0.25 m,  P95 <= 0.50 m")
    verdict_median = "PASS" if median_h <= 0.25 else "FAIL"
    verdict_p95 = "PASS" if p95_h <= 0.50 else "FAIL"
    print(f"  Median  {median_h:.3f} m  ->  {verdict_median}")
    print(f"  P95     {p95_h:.3f} m  ->  {verdict_p95}")
    print("=" * 62)

    if verdict_median == "PASS" and verdict_p95 == "PASS":
        print("\n  Dead reckoning alone already meets the target. ArUco")
        print("  correction becomes a safety net rather than a necessity.")
    else:
        print("\n  Dead reckoning alone does not meet the target. ArUco")
        print("  correction is required. This measurement is the control")
        print("  case to compare the corrected system against.")

    with open(OUTPUT_JSON, "w", encoding="utf-8") as handle:
        json.dump({
            "date": datetime.now().isoformat(timespec="seconds"),
            "mode": mode,
            "duration_s": round(duration, 1),
            "sample_count": len(samples),
            "waypoints_reached": f"{reached}/{total}" if total else None,
            "correction": "none (dead reckoning only)",
            "summary": {
                "horizontal": {
                    "median": round(median_h, 4),
                    "p95": round(p95_h, 4),
                    "max": round(max(h_errors), 4),
                },
                "vertical": {
                    "median": round(statistics.median(v_errors), 4),
                    "p95": round(p95(v_errors), 4),
                    "max": round(max(v_errors), 4),
                },
                "error_3d": {
                    "median": round(statistics.median(e_errors), 4),
                    "p95": round(p95(e_errors), 4),
                    "max": round(max(e_errors), 4),
                },
            },
            "targets": {"median": 0.25, "p95": 0.50},
            "verdict": {"median": verdict_median, "p95": verdict_p95},
            "samples": samples,
        }, handle, indent=2)
    print(f"\n[INFO] Written to {OUTPUT_JSON}")


async def run(hover_seconds):
    global _sampling

    pose_node = trans.Node()
    pose_topic = f"/world/{WORLD}/pose/info"
    pose_node.subscribe(Pose_V, pose_topic, on_pose_info)

    print(f"[INFO] Subscribed to {pose_topic}")
    for _ in range(20):
        if read_ground_truth() is not None:
            break
        await asyncio.sleep(0.25)
    else:
        print("[ERROR] No ground truth received from Gazebo.")
        print(f"        Expected model name '{DRONE_MODEL}' on {pose_topic}")
        return

    drone = System()
    await drone.connect(system_address="udp://:14540")
    print("[INFO] Connecting")
    async for state in drone.core.connection_state():
        if state.is_connected:
            break

    asyncio.create_task(track_position(drone))

    print("[INFO] Waiting for position estimate")
    async for health in drone.telemetry.health():
        if health.is_local_position_ok and health.is_home_position_ok:
            break

    print("[INFO] Arming and taking off")
    await drone.action.arm()
    await drone.action.takeoff()
    await asyncio.sleep(8)

    await drone.offboard.set_position_ned(PositionNedYaw(
        current_pos["n"], current_pos["e"], current_pos["d"], 0.0))
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"[ERROR] Offboard rejected: {error}")
        await drone.action.land()
        return

    start_time = time.time()
    asyncio.create_task(sampler(start_time))

    if hover_seconds:
        print(f"[INFO] Hovering for {hover_seconds} s")
        hold = (current_pos["n"], current_pos["e"], current_pos["d"])
        deadline = time.time() + hover_seconds
        while time.time() < deadline:
            await drone.offboard.set_position_ned(
                PositionNedYaw(hold[0], hold[1], hold[2], 0.0))
            await asyncio.sleep(0.1)
        reached = total = 0
    else:
        route = build_route()
        total = len(route)
        print(f"[INFO] Flying {total} waypoints\n")
        reached = 0
        for index, (x, y, z) in enumerate(route, 1):
            if await goto_waypoint(drone, index, total, x, y, z):
                reached += 1

    duration = time.time() - start_time
    _sampling = False
    await asyncio.sleep(SAMPLE_PERIOD * 2)

    summarize("hover" if hover_seconds else "full_route",
              reached, total, duration)

    print("[INFO] Landing")
    await drone.offboard.stop()
    await drone.action.land()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--hover", type=int, default=0,
                        help="hover for N seconds instead of flying the route")
    args = parser.parse_args()
    asyncio.run(run(args.hover))
