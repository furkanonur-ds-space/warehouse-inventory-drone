"""
Depo-benzeri RL ortami - Gazebo'daki gercek raf duzenimizle AYNI koordinatlar.
Coklu koridor, model kendi kendine koridorlar arasi gecisi ogrenecek
(bizim elle yazdigimiz turn_90/shift_to_next_corridor mantigini biz yazmiyoruz,
model kendi kesfediyor).

Observation: 8 yonlu lidar mesafesi + hedefe mesafe + hedefe aci = 10 sayi
Action:      [ileri_hiz, donus_hizi] (-1..1)
"""
import numpy as np
import gymnasium as gym
from gymnasium import spaces
import pybullet as p
import pybullet_data
import time


class WarehouseEnv(gym.Env):
    metadata = {"render_modes": ["human", "none"]}

    def __init__(self, render_mode="none", start_pos=None):
        super().__init__()
        self.render_mode = render_mode
        # Curriculum learning icin: baslangic konumu disaridan verilebilir
        self.custom_start_pos = np.array(start_pos, dtype=float) if start_pos is not None else None

        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)
        self.n_rays = 8
        self.observation_space = spaces.Box(low=-1.0, high=1.0, shape=(self.n_rays + 2,), dtype=np.float32)

        self.max_speed = 1.2
        self.max_yaw_rate = 60.0
        self.dt = 0.1
        self.max_steps = 500
        self.lidar_max_range = 12.0

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
        p.setGravity(0, 0, 0)
        p.loadURDF("plane.urdf")

        # GERCEK GAZEBO DEPOMUZLA AYNI KOORDINATLAR:
        # Raf siralari y=[-6,-2,2,6], her biri x=-7.5..7.5 arasi kapli (genislik 15m, 3 parca birlesik)
        # Koridorlar arasi gecit: x=±8.5'te (aruco marker koydugumuz yerler)
        self.wall_ids = []
        shelf_half = [7.5, 0.4, 1.0]  # tum sirayi tek kutu olarak temsil ediyoruz
        shelf_rows_y = [-6, -2, 2, 6]
        for y in shelf_rows_y:
            col = p.createCollisionShape(p.GEOM_BOX, halfExtents=shelf_half)
            vis = p.createVisualShape(p.GEOM_BOX, halfExtents=shelf_half, rgbaColor=[0.6, 0.4, 0.2, 1])
            wid = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col,
                                     baseVisualShapeIndex=vis, basePosition=[0, y, 1.0])
            self.wall_ids.append(wid)

        # Dis duvarlar (x=±10, y=±10 - Gazebo ile ayni)
        outer_walls = [
            ([0, 10, 2], [10, 0.3, 2]),
            ([0, -10, 2], [10, 0.3, 2]),
            ([10, 0, 2], [0.3, 10, 2]),
            ([-10, 0, 2], [0.3, 10, 2]),
        ]
        for pos, half in outer_walls:
            col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
            vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=[0.75, 0.75, 0.75, 1])
            wid = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col,
                                     baseVisualShapeIndex=vis, basePosition=pos)
            self.wall_ids.append(wid)

        drone_vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.2, rgbaColor=[0.1, 0.5, 1.0, 1])
        default_start = np.array([-8.0, -8.0, 1.0])
        self.start_pos = self.custom_start_pos.copy() if self.custom_start_pos is not None else default_start
        self.drone_id = p.createMultiBody(baseMass=0, baseVisualShapeIndex=drone_vis,
                                           basePosition=self.start_pos.tolist())

        # Hedef: kutulardan birinin konumu (Gazebo'daki warehouse_box_qr_0 ile ayni yer)
        goal_vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.25, rgbaColor=[0.1, 0.9, 0.1, 0.6])
        self.goal_pos = np.array([-6.0, -5.8, 1.0])
        self.goal_id = p.createMultiBody(baseMass=0, baseVisualShapeIndex=goal_vis,
                                          basePosition=self.goal_pos.tolist())

    def _get_lidar(self):
        """N yonlu raycast (360 derece esit araliklarla)"""
        pos, orn = p.getBasePositionAndOrientation(self.drone_id)
        yaw = p.getEulerFromQuaternion(orn)[2]

        dists = []
        for i in range(self.n_rays):
            a = yaw + (2 * np.pi * i / self.n_rays)
            direction = [np.cos(a), np.sin(a), 0]
            start = pos
            end = [pos[0] + direction[0] * self.lidar_max_range,
                   pos[1] + direction[1] * self.lidar_max_range,
                   pos[2]]
            result = p.rayTest(start, end)[0]
            hit_fraction = result[2]
            dist = hit_fraction * self.lidar_max_range
            dists.append(dist)
        return dists

    def _get_obs(self):
        lidar = self._get_lidar()
        min_dist = min(lidar)
        pos, orn = p.getBasePositionAndOrientation(self.drone_id)
        yaw = p.getEulerFromQuaternion(orn)[2]

        to_goal = self.goal_pos[:2] - np.array(pos[:2])
        dist_to_goal = np.linalg.norm(to_goal)
        angle_to_goal = np.arctan2(to_goal[1], to_goal[0]) - yaw
        angle_to_goal = (angle_to_goal + np.pi) % (2 * np.pi) - np.pi

        lidar_norm = [np.clip(d / self.lidar_max_range, 0, 1) * 2 - 1 for d in lidar]
        obs = np.array(
            lidar_norm + [
                np.clip(dist_to_goal / 25.0, 0, 1) * 2 - 1,
                angle_to_goal / np.pi,
            ],
            dtype=np.float32
        )
        return obs, min_dist, dist_to_goal

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self._build_world()
        self.step_count = 0
        self.pos = self.start_pos.copy()
        self.yaw = 0.0
        obs, _, self.prev_dist_to_goal = self._get_obs()
        return obs, {}

    def step(self, action):
        self.step_count += 1
        forward_speed = float(action[0]) * self.max_speed
        yaw_rate_deg = float(action[1]) * self.max_yaw_rate

        self.yaw += np.radians(yaw_rate_deg) * self.dt
        self.pos[0] += forward_speed * np.cos(self.yaw) * self.dt
        self.pos[1] += forward_speed * np.sin(self.yaw) * self.dt

        quat = p.getQuaternionFromEuler([0, 0, self.yaw])
        p.resetBasePositionAndOrientation(self.drone_id, self.pos.tolist(), quat)

        obs, min_dist, dist_to_goal = self._get_obs()

        reward = 0.0
        terminated = False
        truncated = False

        progress = self.prev_dist_to_goal - dist_to_goal
        reward += progress * 5.0
        reward -= 0.01
        self.prev_dist_to_goal = dist_to_goal

        if min_dist < 0.35:
            reward -= 25.0
            terminated = True

        if dist_to_goal < 0.8:
            reward += 100.0
            terminated = True

        if self.step_count >= self.max_steps:
            truncated = True

        if self.render_mode == "human":
            time.sleep(self.dt)

        return obs, reward, terminated, truncated, {}

    def close(self):
        if self.physics_client is not None:
            p.disconnect(self.physics_client)
