import numpy as np
import gym
from gym import spaces



class Env_1d(gym.Env):

    def __init__(self):
      self.dt = .1
      self.pos = 0.
      self.vel = 0.
      self.acc = 0.
      self.decMax = -5.
      self.accMax = 6.
      self.vMax = 12.
      self.target = 100.
      self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(5,), dtype=np.float64)
      self.action_space = spaces.Box(low=-40, high=40, shape=(1,), dtype=np.float64)

    def config(self, decMax, accMax, vMax, target):
      self.decMax = decMax
      self.accMax = accMax
      self.vMax = vMax
      self.target = target

    def _get_info(self):
      return {"acc": self.acc}

    def _get_obs(self):
      return np.array([self.pos, self.decMax, self.accMax, self.vMax, self.vel], dtype=np.float64)

    def reset(self, seed=None, options=None):
      super().reset(seed=seed)
      self.pos = 0.
      self.vel = 0.
      self.acc = 0.
      return self._get_obs(), self._get_info()

    def step(self, action):
      prev_vel = self.vel
      self.acc = action[0]
      # self.vel = min(self.vel+self.acc*self.dt, self.vMax)
      self.vel = self.vel+self.acc*self.dt
      self.pos += (prev_vel + self.vel)*.5*self.dt
      rew = 0
      # terminated = (self.pos >= self.target)
      terminated = False
      return self._get_obs(), rew, terminated, False, self._get_info()

