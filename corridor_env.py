"""
Basit koridor kacinma ortami (Gymnasium uyumlu).
Drone'u kinematik bir nokta olarak modelliyoruz (tam fizik degil) -
boylece PX4/Gazebo'daki VelocityBodyYawspeed kontrolune birebir denk gelir.

Observation: [on_mesafe, sol_mesafe, sag_mesafe, hedefe_mesafe, hedefe_aci] (5 sayi, normalize)
Action:      [ileri_hiz, donus_hizi] (-1..1 araligi, sonra olceklenir)
Reward:      hedefe yaklasinca +, carpinca buyuk -, adim basina kucuk -
"""
import numpy as np
import gymnasium as gym
from gymnasium import spaces
import pybullet as p
import pybullet_data
import time


class CorridorEnv(gym.Env):
    metadata = {"render_modes": ["human", "none"]}

    def __init__(self, render_mode="none"):
        super().__init__()
        self.render_mode = render_mode

        # Aksiyon: [ileri_hiz(-1..1), donus_hizi(-1..1)]
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)
        # Gozlem: [on, sol, sag, hedef_mesafe, hedef_aci] hepsi normalize -1..1 veya 0..1
        self.observation_space = spaces.Box(low=-1.0, high=1.0, shape=(5,), dtype=np.float32)

        self.max_speed = 1.0        # m/s
        self.max_yaw_rate = 60.0    # derece/s
        self.dt = 0.1                # simulasyon adim suresi
        self.max_steps = 300
        self.lidar_max_range = 8.0

        self.physics_client = None
        self._setup_pybullet()

    def _setup_pybullet(self):
        if self.render_mode == "human":
            self.physics_client = p.connect(p.GUI)
        else:
            self.physics_client = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

    def _build_world(self):
        p.resetSimulation()
        p.setGravity(0, 0, 0)  # kinematik model, yercekimi gerekmiyor
        p.loadURDF("plane.urdf")

        # Basit koridor: iki sira duvar (kutu), aralarinda gecit
        self.wall_ids = []
        wall_half = [3.0, 0.2, 1.0]  # uzunluk, kalinlik, yukseklik/2

        # Sol duvar sirasi (birkac parca, arasinda bosluklar - koridor kapilari)
        wall_positions = [
            (0, 2.0, 1.0), (0, -2.0, 1.0),      # ilk koridor
            (8, 2.0, 1.0), (8, -2.0, 1.0),      # ikinci koridor
        ]
        for (x, y, z) in wall_positions:
            col = p.createCollisionShape(p.GEOM_BOX, halfExtents=wall_half)
            vis = p.createVisualShape(p.GEOM_BOX, halfExtents=wall_half, rgbaColor=[0.6, 0.4, 0.2, 1])
            wid = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col,
                                     baseVisualShapeIndex=vis, basePosition=[x, y, z])
            self.wall_ids.append(wid)

        # Drone temsili (kucuk kure, gorsel amacli)
        drone_vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.15, rgbaColor=[0.1, 0.5, 1.0, 1])
        self.drone_id = p.createMultiBody(baseMass=0, baseVisualShapeIndex=drone_vis,
                                           basePosition=[-3, 0, 1])

        # Hedef (yesil kure, gorsel amacli)
        goal_vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.2, rgbaColor=[0.1, 0.9, 0.1, 0.6])
        self.goal_pos = np.array([12.0, 0.0, 1.0])
        self.goal_id = p.createMultiBody(baseMass=0, baseVisualShapeIndex=goal_vis,
                                          basePosition=self.goal_pos.tolist())

    def _get_lidar(self):
        """Basit 3 yonlu raycast: on, sol, sag"""
        pos, orn = p.getBasePositionAndOrientation(self.drone_id)
        yaw = p.getEulerFromQuaternion(orn)[2]

        angles = [yaw, yaw + np.pi / 2, yaw - np.pi / 2]  # on, sol, sag
        dists = []
        for a in angles:
            direction = [np.cos(a), np.sin(a), 0]
            start = pos
            end = [pos[0] + direction[0] * self.lidar_max_range,
                   pos[1] + direction[1] * self.lidar_max_range,
                   pos[2]]
            result = p.rayTest(start, end)[0]
            hit_fraction = result[2]
            dist = hit_fraction * self.lidar_max_range
            dists.append(dist)
        return dists  # [on, sol, sag]

    def _get_obs(self):
        front, left, right = self._get_lidar()
        pos, orn = p.getBasePositionAndOrientation(self.drone_id)
        yaw = p.getEulerFromQuaternion(orn)[2]

        to_goal = self.goal_pos[:2] - np.array(pos[:2])
        dist_to_goal = np.linalg.norm(to_goal)
        angle_to_goal = np.arctan2(to_goal[1], to_goal[0]) - yaw
        angle_to_goal = (angle_to_goal + np.pi) % (2 * np.pi) - np.pi  # -pi..pi araligina normalize

        obs = np.array([
            np.clip(front / self.lidar_max_range, 0, 1) * 2 - 1,
            np.clip(left / self.lidar_max_range, 0, 1) * 2 - 1,
            np.clip(right / self.lidar_max_range, 0, 1) * 2 - 1,
            np.clip(dist_to_goal / 15.0, 0, 1) * 2 - 1,
            angle_to_goal / np.pi,
        ], dtype=np.float32)
        return obs, front, dist_to_goal

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self._build_world()
        self.step_count = 0
        self.pos = np.array([-3.0, 0.0, 1.0])
        self.yaw = 0.0
        obs, _, self.prev_dist_to_goal = self._get_obs()
        return obs, {}

    def step(self, action):
        self.step_count += 1
        forward_speed = float(action[0]) * self.max_speed
        yaw_rate_deg = float(action[1]) * self.max_yaw_rate

        # Kinematik entegrasyon (PX4 VelocityBodyYawspeed mantigiyla ayni)
        self.yaw += np.radians(yaw_rate_deg) * self.dt
        self.pos[0] += forward_speed * np.cos(self.yaw) * self.dt
        self.pos[1] += forward_speed * np.sin(self.yaw) * self.dt

        quat = p.getQuaternionFromEuler([0, 0, self.yaw])
        p.resetBasePositionAndOrientation(self.drone_id, self.pos.tolist(), quat)

        obs, front, dist_to_goal = self._get_obs()

        # ODUL HESABI
        reward = 0.0
        terminated = False
        truncated = False

        progress = self.prev_dist_to_goal - dist_to_goal
        reward += progress * 5.0          # hedefe yaklasinca odul
        reward -= 0.02                     # zaman cezasi (hizli olsun diye)
        self.prev_dist_to_goal = dist_to_goal

        if front < 0.3:                    # carpma
            reward -= 20.0
            terminated = True

        if dist_to_goal < 0.6:             # hedefe ulasti
            reward += 50.0
            terminated = True

        if self.step_count >= self.max_steps:
            truncated = True

        if self.render_mode == "human":
            time.sleep(self.dt)

        return obs, reward, terminated, truncated, {}

    def close(self):
        if self.physics_client is not None:
            p.disconnect(self.physics_client)
