import gymnasium as gym
import highway_env
from stable_baselines3 import DQN
from highway_env.envs import ControlledVehicle, Vehicle
from highway_env.envs.common.observation import KinematicObservation

env = gym.make("highway-fast-v0", render_mode="rgb_array")
KinematicObservation.normalize_obs = lambda self, df: df # Don't normalize values
env.config['lanes_count']=4

env.config['observation']={
    'type': 'Kinematics',
    'vehicles_count': 10,
    'features': ['presence', 'x', 'y', 'vx', 'vy', 'heading'],
    'absolute': True
}

env.reset()

model = DQN.load("highway_dqn/model")
model.set_env(env)

# Load and test saved model
while True:
  done = truncated = False
  obs, info = env.reset()
  while not (done or truncated):
    action, _states = model.predict(obs, deterministic=True)
    obs, reward, done, truncated, info = env.step(action)
    env.render()