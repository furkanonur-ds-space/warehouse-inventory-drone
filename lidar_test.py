import os
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"

import gz.transport13 as trans
from gz.msgs10.laserscan_pb2 import LaserScan
import time

LIDAR_TOPIC = "/world/warehouse/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan"

latest_scan = None

def on_scan(msg):
    global latest_scan
    latest_scan = msg

def main():
    node = trans.Node()
    node.subscribe(LaserScan, LIDAR_TOPIC, on_scan)

    print("Lidar dinleniyor... (Ctrl+C ile durdur)")
    print(f"Topic: {LIDAR_TOPIC}\n")

    while True:
        if latest_scan is not None:
            ranges = list(latest_scan.ranges)
            n = len(ranges)

            # Lidar 270 derece, 1080 nokta -> merkez index = sol/sag/on
            front_idx = n // 2          # tam on (0 derece)
            left_idx  = n // 4          # sol (yaklasik -67.5 derece)
            right_idx = 3 * n // 4      # sag (yaklasik +67.5 derece)

            front_dist = ranges[front_idx]
            left_dist  = ranges[left_idx]
            right_dist = ranges[right_idx]

            print(f"ON: {front_dist:5.2f}m  |  SOL: {left_dist:5.2f}m  |  SAG: {right_dist:5.2f}m")

        time.sleep(0.5)

if __name__ == "__main__":
    main()
