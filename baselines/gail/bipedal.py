import gym
env = gym.make("BipedalWalker-v3")

env.reset()
while True:
  obs = env.step([-.5, .5, .5, .5])
  print(obs)

  