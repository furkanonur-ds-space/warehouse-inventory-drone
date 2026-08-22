"""Run the trained model with the GUI open, so the behaviour can be watched."""
from corridor_env import CorridorEnv
from stable_baselines3 import PPO
import time

model = PPO.load("corridor_models/best_model")

env = CorridorEnv(render_mode="human")
obs, info = env.reset()

print("Egitilmis model calisiyor, izleyin...")
total_reward = 0
for i in range(300):
    action, _states = model.predict(obs, deterministic=True)
    obs, reward, terminated, truncated, info = env.step(action)
    total_reward += reward

    if terminated or truncated:
        print(f"Bolum bitti. Toplam odul: {total_reward:.2f}")
        if terminated and reward > 0:
            print("HEDEFE ULASTI!")
        elif terminated:
            print("CARPTI")
        break

time.sleep(2)
env.close()
