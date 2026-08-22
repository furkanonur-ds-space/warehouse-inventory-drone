"""
Kademeli (curriculum) RL egitimi.
Starts with an easy task (close to the goal) and makes it harder on success.
Each stage continues from the model learned in the previous one (transfer).
"""
import os
from warehouse_rl_env import WarehouseEnv
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import EvalCallback

MODEL_DIR = "curriculum_models"
os.makedirs(MODEL_DIR, exist_ok=True)

GOAL = (-6.0, -5.8)  # hedef konumu (degismiyor)

# Per-stage starting points, from easiest to hardest.
# The stage names are also the directory names under curriculum_models/,
# so they are left as they are: renaming them orphans the trained models.
STAGES = [
    {"name": "Asama1_Cok_Yakin",        "start": (-4.0, -5.8), "steps": 40_000},
    {"name": "Asama2_Ayni_Koridor",     "start": (-6.0, -4.0), "steps": 60_000},
    {"name": "Asama3_Koridor_Sonu",     "start": (-8.0, -4.0), "steps": 80_000},
    {"name": "Asama4_Komsu_Koridor",    "start": (-8.0, 0.0),  "steps": 100_000},
    {"name": "Asama5_Tam_Zor",          "start": (-8.0, -8.0), "steps": 150_000},
]

model = None

for i, stage in enumerate(STAGES):
    print(f"\n{'='*60}")
    print(f"  {stage['name']} starting - start point: {stage['start']}")
    print(f"{'='*60}\n")

    train_env = make_vec_env(
        lambda: WarehouseEnv(render_mode="none", start_pos=(*stage["start"], 1.0)),
        n_envs=1
    )
    eval_env = WarehouseEnv(render_mode="none", start_pos=(*stage["start"], 1.0))

    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path=os.path.join(MODEL_DIR, stage["name"]),
        log_path=os.path.join(MODEL_DIR, stage["name"]),
        eval_freq=5000,
        n_eval_episodes=5,
        deterministic=True,
    )

    if model is None:
        # First stage: create a model from scratch
        model = PPO("MlpPolicy", train_env, verbose=1, learning_rate=3e-4,
                     n_steps=2048, batch_size=64)
    else:
        # Continue from the previous stage (transfer learning)
        model.set_env(train_env)

    model.learn(total_timesteps=stage["steps"], callback=eval_callback, reset_num_timesteps=False)

    stage_model_path = os.path.join(MODEL_DIR, f"{stage['name']}_final")
    model.save(stage_model_path)
    print(f"\n{stage['name']} complete. Saved to: {stage_model_path}.zip")

# Save the last model as the main model
model.save(os.path.join(MODEL_DIR, "final_curriculum_model"))
print(f"\n{'='*60}")
print("TUM CURRICULUM TAMAMLANDI!")
print(f"Final model: {MODEL_DIR}/final_curriculum_model.zip")
print(f"{'='*60}")
