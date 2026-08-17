"""
TEMEL UCUS TESTI: Optical flow (GPS'siz) ile drone kalkip stabil durabiliyor mu?
Bu, sonraki adimlarin (waypoint navigasyon) uzerine insa edilecegi temel.
"""
import os
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"

import asyncio
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed, OffboardError


async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("[BILGI] Baglaniliyor...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            break
    print("[BILGI] Baglanti kuruldu.")

    # Saglik durumunu izle - ozellikle local_position_ok (optical flow'a bagli)
    print("[BILGI] Sensor sagligi bekleniyor (optical flow / EKF2 stabilize olmali)...")
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
        if timeout_counter > 60:  # ~30 saniye (health stream ~0.5s araliklarla gelir)
            print("[UYARI] 30 saniyede sensor hazir olmadi, yine de devam edilecek")
            break

    print(f"\n[BILGI] Sensor durumu: {'HAZIR' if sensors_ready else 'HAZIR DEGIL (riskli devam)'}")

    print("[BILGI] Motorlar calistiriliyor...")
    try:
        await drone.action.arm()
    except Exception as e:
        print(f"[HATA] Arm basarisiz: {e}")
        return

    print("[BILGI] Kalkis...")
    await drone.action.takeoff()

    print("[BILGI] 15 saniye boyunca irtifa ve konum izleniyor (stabil mi?)...")
    for i in range(15):
        async for pos in drone.telemetry.position():
            alt = pos.relative_altitude_m
            break
        async for odom in drone.telemetry.position_velocity_ned():
            vx = odom.velocity.north_m_s
            vy = odom.velocity.east_m_s
            break
        print(f"  t={i}s  irtifa={alt:.2f}m  hiz_north={vx:.2f}  hiz_east={vy:.2f}")
        await asyncio.sleep(1)

    print("\n[BILGI] Offboard moda geciliyor (sabit durma testi)...")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"[HATA] Offboard baslatilamadi: {e}")
        await drone.action.land()
        return

    print("[BILGI] 10 saniye sabit durma (drift olcumu)...")
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
        print(f"\n[SONUC] 10 saniyede toplam drift: {drift:.3f} metre")
        print(f"  Baslangic: N={start_n:.2f} E={start_e:.2f}")
        print(f"  Bitis:     N={end_n:.2f} E={end_e:.2f}")

    print("\n[BILGI] Test tamamlandi, iniliyor...")
    await drone.offboard.stop()
    await drone.action.land()


if __name__ == "__main__":
    asyncio.run(run())
