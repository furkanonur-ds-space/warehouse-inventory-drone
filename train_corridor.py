"""Corridor ortaminda PPO ile RL egitimi yapar"""
import os
from corridor_env import CorridorEnv
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import EvalCallback

MODEL_DIR = "corridor_models"
os.makedirs(MODEL_DIR, exist_ok=True)

# Egitim ortami (headless, hizli)
train_env = make_vec_env(lambda: CorridorEnv(render_mode="none"), n_envs=1)

# Degerlendirme ortami (egitim sirasinda periyodik test icin)
eval_env = CorridorEnv(render_mode="none")

eval_callback = EvalCallback(
    eval_env,
    best_model_save_path=MODEL_DIR,
    log_path=MODEL_DIR,
    eval_freq=2000,
    n_eval_episodes=5,
    deterministic=True,
)

model = PPO(
    "MlpPolicy",
    train_env,
    verbose=1,
    learning_rate=3e-4,
    n_steps=1024,
    batch_size=64,
)

print("Egitim basliyor... (bu birkac dakika surebilir)")
model.learn(total_timesteps=100_000, callback=eval_callback)

model.save(os.path.join(MODEL_DIR, "final_model"))
print(f"\nEgitim tamamlandi! Model kaydedildi: {MODEL_DIR}/final_model.zip")
