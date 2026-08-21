"""
Performance probe: measure the simulation's real time factor and camera frame
rates without flying anything.

Run this with the simulation up but no mission script running, then again with
the mission running, to see what the scan actually costs.

Usage:
    python3 perf_probe.py            # 20 second sample
    python3 perf_probe.py --secs 60
"""
import os
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"

import argparse
import statistics
import time

import gz.transport13 as trans
from gz.msgs10.image_pb2 import Image
from gz.msgs10.world_stats_pb2 import WorldStatistics

WORLD = "warehouse_v2"
DRONE = "x500_scanner_0"

CAMERAS = {
    "left":  f"/world/{WORLD}/model/{DRONE}/link/camera_left_link/sensor/camera/image",
    "right": f"/world/{WORLD}/model/{DRONE}/link/camera_right_link/sensor/camera/image",
    "down":  f"/world/{WORLD}/model/{DRONE}/link/camera_down_link/sensor/camera/image",
}
STATS_TOPIC = f"/world/{WORLD}/stats"

frame_times = {name: [] for name in CAMERAS}
frame_sizes = {name: None for name in CAMERAS}
rtf_samples = []


def make_camera_callback(name):
    def callback(msg):
        frame_times[name].append(time.time())
        if frame_sizes[name] is None:
            frame_sizes[name] = (msg.width, msg.height)
    return callback


def on_stats(msg):
    rtf_samples.append(msg.real_time_factor)


def summarize(duration):
    print("\n" + "=" * 62)
    print("  SIMULATION PERFORMANCE")
    print(f"  Sample window: {duration} s")
    print("=" * 62)

    if rtf_samples:
        print(f"\n  Real time factor")
        print(f"    median {statistics.median(rtf_samples):.3f}")
        print(f"    mean   {statistics.mean(rtf_samples):.3f}")
        print(f"    min    {min(rtf_samples):.3f}")
        print(f"    max    {max(rtf_samples):.3f}")
        below = sum(1 for r in rtf_samples if r < 0.9)
        print(f"    below 0.9 in {below}/{len(rtf_samples)} samples "
              f"({100*below/len(rtf_samples):.0f}%)")
        print("\n    1.0 means simulated time keeps up with wall clock.")
        print("    Sustained values below about 0.9 cause PX4 timestamp errors")
        print("    and dropped camera frames.")
    else:
        print("\n  No world statistics received.")

    print(f"\n  Camera frame rates")
    for name, times in frame_times.items():
        if len(times) < 2:
            print(f"    {name:6s} no frames")
            continue
        span = times[-1] - times[0]
        fps = (len(times) - 1) / span if span > 0 else 0
        gaps = [times[i+1] - times[i] for i in range(len(times) - 1)]
        w, h = frame_sizes[name] or (0, 0)
        print(f"    {name:6s} {w}x{h}  {fps:5.1f} fps   "
              f"median gap {statistics.median(gaps)*1000:5.1f} ms   "
              f"worst gap {max(gaps)*1000:6.1f} ms")

    print("\n" + "=" * 62)


def main(secs):
    nodes = []
    for name, topic in CAMERAS.items():
        node = trans.Node()
        if node.subscribe(Image, topic, make_camera_callback(name)):
            nodes.append(node)
        else:
            print(f"[WARN] could not subscribe to {name} camera")

    stats_node = trans.Node()
    if not stats_node.subscribe(WorldStatistics, STATS_TOPIC, on_stats):
        print(f"[WARN] could not subscribe to {STATS_TOPIC}")
    nodes.append(stats_node)

    print(f"[INFO] Sampling for {secs} s")
    time.sleep(secs)
    summarize(secs)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--secs", type=int, default=20)
    args = parser.parse_args()
    main(args.secs)
