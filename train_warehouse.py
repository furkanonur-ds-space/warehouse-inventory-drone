"""Depo ortaminda PPO ile RL egitimi"""
import os
from warehouse_rl_env import WarehouseEnv
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import EvalCallback

MODEL_DIR = "warehouse_models"
os.makedirs(MODEL_DIR, exist_ok=True)

train_env = make_vec_env(lambda: WarehouseEnv(render_mode="none"), n_envs=1)
eval_env = WarehouseEnv(render_mode="none")

eval_callback = EvalCallback(
    eval_env,
    best_model_save_path=MODEL_DIR,
    log_path=MODEL_DIR,
    eval_freq=5000,
    n_eval_episodes=5,
    deterministic=True,
)

model = PPO(
    "MlpPolicy",
    train_env,
    verbose=1,
    learning_rate=3e-4,
    n_steps=2048,
    batch_size=64,
)

print("Depo ortaminda egitim basliyor... (bu daha uzun surebilir - koridor gecisi zor bir gorev)")
model.learn(total_timesteps=400_000, callback=eval_callback)

model.save(os.path.join(MODEL_DIR, "final_model"))
print(f"\nEgitim tamamlandi! Model kaydedildi: {MODEL_DIR}/final_model.zip")
