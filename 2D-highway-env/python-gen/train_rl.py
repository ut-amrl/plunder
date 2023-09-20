import gymnasium as gym
import highway_env
from stable_baselines3 import DQN
from highway_env.envs import ControlledVehicle, Vehicle
from highway_env.envs.common.observation import KinematicObservation

env = gym.make("highway-fast-v0")
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

# model = DQN('MlpPolicy', env,
#               policy_kwargs=dict(net_arch=[256, 256]),
#               learning_rate=5e-4,
#               buffer_size=15000,
#               learning_starts=200,
#               batch_size=32,
#               gamma=0.8,
#               train_freq=1,
#               gradient_steps=1,
#               target_update_interval=50,
#               verbose=1,
#               tensorboard_log="highway_dqn/")
model.learn(int(3e5))
model.save("highway_dqn/model")