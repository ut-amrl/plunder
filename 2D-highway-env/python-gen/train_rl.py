import gymnasium as gym
import highway_env
from stable_baselines3 import DQN
from highway_env.envs import ControlledVehicle, Vehicle
from highway_env.envs.common.observation import KinematicObservation
import numpy as np
import random
from typing import Union

env = gym.make("highway-fast-v0")

######## Configuration ########
lanes_count = 4 # Number of lanes
KinematicObservation.normalize_obs = lambda self, df: df # Don't normalize values

env.config['lanes_count']=lanes_count
env.config['observation']={
    'type': 'Kinematics',
    'vehicles_count': 10,
    'features': ['presence', 'x', 'y', 'vx', 'vy', 'heading'],
    'absolute': True
}

env.reset()
model = DQN('MlpPolicy', env,
              policy_kwargs=dict(net_arch=[256, 256]),
              learning_rate=5e-4,
              buffer_size=15000,
              learning_starts=200,
              batch_size=32,
              gamma=0.8,
              train_freq=1,
              gradient_steps=1,
              target_update_interval=50,
              verbose=1,
              tensorboard_log="highway_dqn/")
model.learn(200000)
model.save("highway_dqn/model")

# Load and test saved model
# model = DQN.load("highway_dqn/model")
# while True:
#   done = truncated = False
#   obs, info = env.reset()
#   while not (done or truncated):
#     # action, _states = model.predict(obs, deterministic=True)
#     action = 0
#     obs, reward, done, truncated, info = env.step(action)
#     env.render()