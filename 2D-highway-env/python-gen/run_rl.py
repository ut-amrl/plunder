import gymnasium as gym
import highway_env
from stable_baselines3 import DQN

env = gym.make("highway-fast-v0", render_mode="rgb_array")

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