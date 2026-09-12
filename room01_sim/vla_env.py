"""Gymnasium interface for VLA rollout evaluation in the fixed workcell.

The application must already be running. Policy inputs contain images,
proprioception and an instruction; simulator object poses are kept in info.
"""
import string

import gymnasium as gym
import numpy as np

from room01_sim.episodes import policy_observation
from room01_sim.runtime import RoomTaskSim


class Room01VLAEnv(gym.Env):
    metadata = {"render_modes": ["rgb_array"], "render_fps": 30}

    def __init__(self, application, *, device="cuda:0", seed=0):
        self.task = RoomTaskSim(application, device=device, cameras=True, seed=seed)
        config = self.task.config
        self.action_space = gym.spaces.Box(self.task.action_lower, self.task.action_upper, dtype=np.float32)
        self.observation_space = gym.spaces.Dict({
            "state": gym.spaces.Box(-np.inf, np.inf, (26,), np.float32),
            "joint_velocity": gym.spaces.Box(-np.inf, np.inf, (26,), np.float32),
            "images": gym.spaces.Dict({name: gym.spaces.Box(0, 255, (spec["resolution"][1], spec["resolution"][0], 3), np.uint8)
                                       for name, spec in config["cameras"].items()}),
            "language_instruction": gym.spaces.Text(max_length=4096, min_length=0, charset=set(string.printable+config["language_instruction"])),
            "timestamp": gym.spaces.Box(0., np.inf, (), np.float64),
        })
        self._observation = None

    def _public_observation(self, observation):
        public = policy_observation(observation)
        public["timestamp"] = np.asarray(public["timestamp"], np.float64)
        self._observation = public
        return public

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        _, info = self.task.reset(seed=seed, randomize=(options or {}).get("randomize", True))
        observation = self.task.observe(images=True)
        info["camera_metadata"] = observation["camera_metadata"]
        return self._public_observation(observation), info

    def step(self, action):
        observation, reward, terminated, truncated, info = self.task.step(action, images=True)
        info["ground_truth"] = {"cube_pose_xyzw": observation["cube_pose_xyzw"]}
        info["camera_metadata"] = observation["camera_metadata"]
        return self._public_observation(observation), reward, terminated, truncated, info

    def render(self):
        if self._observation is None:
            return None
        return self._observation["images"]["head"].copy()

    def close(self):
        self.task.close()
