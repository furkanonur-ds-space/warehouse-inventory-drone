"""Corridor ortamini rastgele aksiyonlarla test eder (egitim yok, sadece dogrulama)"""
from corridor_env import CorridorEnv

env = CorridorEnv(render_mode="none")
obs, info = env.reset()
print(f"Ilk gozlem: {obs}")
print(f"Gozlem sekli: {env.observation_space}")
print(f"Aksiyon sekli: {env.action_space}")

total_reward = 0
for i in range(50):
    action = env.action_space.sample()  # rastgele aksiyon
    obs, reward, terminated, truncated, info = env.step(action)
    total_reward += reward
    if i % 10 == 0:
        print(f"Adim {i}: obs={obs.round(2)} reward={reward:.2f}")
    if terminated or truncated:
        print(f"Bolum bitti (adim {i}): terminated={terminated} truncated={truncated}")
        break

print(f"\nToplam odul: {total_reward:.2f}")
print("Ortam basariyla calisti!")
env.close()
