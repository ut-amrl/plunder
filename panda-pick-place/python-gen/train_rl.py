import gymnasium as gym
import panda_gym
from stable_baselines3 import PPO
import time

env = gym.make("PandaPickAndPlaceDense-v3")
model = PPO(policy="MultiInputPolicy", env=env)
model.learn(100_000)

model.save("ddpg-pick-and-place")
