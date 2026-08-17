import os
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"

import asyncio
import socket
import importlib.util
import sys

# UDP sunucusu
udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
def send_intel(msg):
    udp_sock.sendto(f"[MERKEZ] {msg}".encode("utf-8"), ("127.0.0.1", 5005))
    print(f"[MERKEZ] {msg}")

def load_drone_module(idx):
    spec = importlib.util.spec_from_file_location(
        f"swarm_drone_{idx}",
        "/home/furk/autonomous_landing/swarm_drone.py"
    )
    mod = importlib.util.module_from_spec(spec)
    
    original_argv = sys.argv.copy()
    sys.argv = ["swarm_drone.py", str(idx)]
    
    spec.loader.exec_module(mod)
    
    sys.argv = original_argv
    return mod

async def main():
    send_intel("Suru operasyonu baslatiliyor...")

    # 1. Aşama: Yerde Senkronizasyon (Motor calistirma ve kalkis)
    drone0_ready = asyncio.Event()
    drone1_ready = asyncio.Event()
    start_event  = asyncio.Event()

    # 2. Aşama: Havada Senkronizasyon (Offboard'a gecis oncesi hizalanma)
    takeoff_drone0_ready = asyncio.Event()
    takeoff_drone1_ready = asyncio.Event()
    takeoff_start_event  = asyncio.Event()

    async def wait_and_start():
        await drone0_ready.wait()
        await drone1_ready.wait()
        send_intel("Her iki drone hazir! SENKRONIZE KALKIS KOMUTU VERILDI!")
        start_event.set()

    async def wait_and_sync_midair():
        await takeoff_drone0_ready.wait()
        await takeoff_drone1_ready.wait()
        send_intel("Her iki drone ayni irtifada! OFFBOARD DEVRIYE BASLATILIYOR!")
        takeoff_start_event.set()

    send_intel("Drone modulleri izole edilerek yukleniyor...")
    
    drone0_mod = load_drone_module(0)
    drone1_mod = load_drone_module(1)

    send_intel("Drone'lar baslatiliyor...")
    
    await asyncio.gather(
        drone0_mod.run(drone0_ready, start_event, takeoff_drone0_ready, takeoff_start_event),
        drone1_mod.run(drone1_ready, start_event, takeoff_drone1_ready, takeoff_start_event),
        wait_and_start(),
        wait_and_sync_midair()
    )
    send_intel("TUM DRONLAR GOREVI TAMAMLADI!")

if __name__ == "__main__":
    asyncio.run(main())
