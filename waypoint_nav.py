"""
ADIM 2: Waypoint tabanli navigasyon testi (kamera/tarama YOK, sadece hareket).
Depo krokisi onceden bilindigi icin sabit bir rota izliyoruz.

Rota deseni (boustrophedon / zigzag):
  Koridor 1, seviye 1: guneyden kuzeye
  Koridor 1, seviye 2: kuzeyden guneye
  Koridor 1, seviye 3: guneyden kuzeye
  Koridor 2'ye gec, seviye 3'ten basla (asagi dogru)
  ...
"""
import os
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"

import asyncio
import math
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError

# ─── DEPO KROKISI (onceden bilinen) ─────────────────────────
CORRIDOR_X = [-8.5, -3.9, 0.7, 6.9]     # koridor merkezleri
FLIGHT_Z = [0.5, 1.15, 1.8]              # ucus irtifalari (raf seviyelerinin biraz ustu)
Y_SOUTH, Y_NORTH = -8.0, 8.0             # koridor uclari (duvara biraz mesafe birakiyoruz)

# Test icin sadece ilk 2 koridoru tarayalim (sure kisa olsun)
CORRIDORS_TO_SCAN = 2

WAYPOINT_TOLERANCE = 0.4   # metre - bu mesafeye girince "vardim" say
TIMEOUT_PER_WP = 40         # saniye - bir waypoint icin maksimum sure


def build_route():
    """Boustrophedon rota olusturur: (x, y, z, yaw) listesi"""
    route = []
    going_north = True

    for c_idx in range(CORRIDORS_TO_SCAN):
        x = CORRIDOR_X[c_idx]
        # Seviye sirasi: ilk koridorda alttan uste, ikincide ustten alta...
        levels = FLIGHT_Z if c_idx % 2 == 0 else list(reversed(FLIGHT_Z))

        for z in levels:
            if going_north:
                route.append((x, Y_SOUTH, z, 0.0))   # baslangic noktasi
                route.append((x, Y_NORTH, z, 0.0))   # koridoru gec
            else:
                route.append((x, Y_NORTH, z, 0.0))
                route.append((x, Y_SOUTH, z, 0.0))
            going_north = not going_north

    return route


# ─── DURUM ──────────────────────────────────────────────────
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
    Waypoint'e git. NOT: PX4 NED koordinat sistemi kullaniyor:
      North = Gazebo X, East = Gazebo Y, Down = -Gazebo Z
    Ama drone spawn noktasi NED origin oldugu icin, spawn'a gore RELATIF hesapliyoruz.
    """
    # Gazebo dunya koordinatlarini, spawn noktasina gore NED'e cevir
    # Spawn: x=-8.5, y=-9, yaw=90deg  -> bu noktayi origin kabul ediyoruz
    SPAWN_X, SPAWN_Y = -8.5, -9.0

    # Drone yaw=90 derece ile spawn oldugu icin, Gazebo X ekseni -> NED East'e,
    # Gazebo Y ekseni -> NED North'a denk geliyor
    target_n = y - SPAWN_Y
    target_e = x - SPAWN_X
    target_d = -z    # NED'de asagi pozitif, yukseklik negatif

    print(f"\n[WP {wp_idx}/{total}] Hedef: Gazebo(x={x:.1f}, y={y:.1f}, z={z:.2f})")
    print(f"           NED karsiligi: N={target_n:.2f} E={target_e:.2f} D={target_d:.2f}")

    elapsed = 0.0
    step = 0.2
    while elapsed < TIMEOUT_PER_WP:
        await drone.offboard.set_position_ned(
            PositionNedYaw(target_n, target_e, target_d, yaw))

        dist = distance_to(target_n, target_e, target_d)
        if dist < WAYPOINT_TOLERANCE:
            print(f"           ✓ VARDI (hata: {dist:.2f}m, sure: {elapsed:.1f}s)")
            return True

        if int(elapsed * 5) % 25 == 0:  # her 5 saniyede bir durum
            print(f"           ... mesafe: {dist:.2f}m  konum: N={current_pos['n']:.2f} E={current_pos['e']:.2f} D={current_pos['d']:.2f}")

        await asyncio.sleep(step)
        elapsed += step

    print(f"           ✗ ZAMAN ASIMI (kalan mesafe: {distance_to(target_n, target_e, target_d):.2f}m)")
    return False


async def run():
    route = build_route()
    print("="*60)
    print(f"  WAYPOINT NAVIGASYON TESTI")
    print(f"  Toplam waypoint: {len(route)}")
    print(f"  Taranacak koridor: {CORRIDORS_TO_SCAN}")
    print("="*60)

    drone = System()
    await drone.connect(system_address="udp://:14540")
    print("\n[BILGI] Baglaniliyor...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            break

    asyncio.create_task(track_position(drone))

    print("[BILGI] Sensorler bekleniyor...")
    async for h in drone.telemetry.health():
        if h.is_local_position_ok and h.is_home_position_ok:
            break
    print("[BILGI] Sensorler hazir.")

    print("[BILGI] Arm ve kalkis...")
    await drone.action.arm()
    await drone.action.takeoff()
    await asyncio.sleep(8)

    print("[BILGI] Offboard moda geciliyor...")
    await drone.offboard.set_position_ned(
        PositionNedYaw(current_pos["n"], current_pos["e"], current_pos["d"], 0.0))
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"[HATA] Offboard: {e}")
        await drone.action.land()
        return

    print("\n[BILGI] Rota takibi basliyor...\n")
    success_count = 0
    for i, (x, y, z, yaw) in enumerate(route, 1):
        ok = await goto_waypoint(drone, i, len(route), x, y, z, yaw)
        if ok:
            success_count += 1

    print("\n" + "="*60)
    print(f"  ROTA TAMAMLANDI")
    print(f"  Basarili waypoint: {success_count}/{len(route)}")
    print("="*60)

    print("\n[BILGI] Iniliyor...")
    await drone.offboard.stop()
    await drone.action.land()


if __name__ == "__main__":
    asyncio.run(run())
