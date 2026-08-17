"""Egitilmis depo modelini GUI'de izler"""
from warehouse_rl_env import WarehouseEnv
from stable_baselines3 import PPO
import time

model = PPO.load("warehouse_models/best_model")

env = WarehouseEnv(render_mode="human")
obs, info = env.reset()

print("Egitilmis depo modeli calisiyor, izleyin...")
total_reward = 0
for i in range(500):
    action, _states = model.predict(obs, deterministic=True)
    obs, reward, terminated, truncated, info = env.step(action)
    total_reward += reward

    if terminated or truncated:
        print(f"Bolum bitti (adim {i}). Toplam odul: {total_reward:.2f}")
        if terminated and reward > 50:
            print("HEDEFE ULASTI! (kutuyu buldu)")
        elif terminated:
            print("CARPTI")
        else:
            print("ZAMAN ASIMI (hedefe ulasamadi)")
        break

time.sleep(3)
env.close()
