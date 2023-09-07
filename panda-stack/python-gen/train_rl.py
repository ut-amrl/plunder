import gymnasium as gym
import panda_gym
from stable_baselines3 import PPO, A2C, DDPG
from sb3_contrib import TQC
import time

algo = "TQC"
from_existing = True
learn_time = 1_000_000

env = gym.make("PandaStackDense-v3")

if algo == "DDPG":
    if from_existing: 
        model = DDPG.load("ddpg-stack", env=env)
    else:
        model = DDPG(policy="MultiInputPolicy", env=env)
    model.learn(learn_time, progress_bar=True)
    model.save("ddpg-stack-new")

elif algo == "A2C":
    if from_existing: 
        model = A2C.load("a2c-stack", env=env)
    else:
        model = A2C(policy="MultiInputPolicy", env=env)
    model.learn(learn_time, progress_bar=True)
    model.save("a2c-stack-new")

elif algo == "PPO":
    if from_existing: 
        model = PPO.load("ppo-stack", env=env)
    else:
        model = PPO(policy="MultiInputPolicy", env=env)
    model.learn(learn_time, progress_bar=True)
    model.save("ppo-stack-new")

elif algo == "TQC":
    if from_existing: 
        model = TQC.load("tqc-stack", env=env)
    else:
        model = TQC(policy="MultiInputPolicy", env=env)
    model.learn(learn_time, progress_bar=True)
    model.save("tqc-stack-new")

