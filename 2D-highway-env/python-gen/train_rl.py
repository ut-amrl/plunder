import gymnasium as gym
import highway_env
from stable_baselines3 import DQN
from highway_env.envs import ControlledVehicle, Vehicle
from highway_env.envs.common.observation import KinematicObservation
import numpy as np
import random
from typing import Union

env = gym.make("highway-v0", render_mode='rgb_array')

######## Configuration ########
lane_diff = 4 # Distance lanes are apart from each other
lanes_count = 4 # Number of lanes
use_absolute_lanes = True # Whether or not to label lanes as absolute or relative to current vehicle lane
KinematicObservation.normalize_obs = lambda self, df: df # Don't normalize values

env.config['simulation_frequency']=24
env.config['policy_frequency']=8 # Runs once every 3 simulation steps
env.config['lanes_count']=lanes_count

# Observations
# ego vehicle:      presence, x, y, vx, vy, heading
# 9 other vehicles: presence, x, y, vx, vy, heading        (x and y relative to ego)
env.config['observation']={
    'type': 'Kinematics',
    'vehicles_count': 10,
    'features': ['presence', 'x', 'y', 'vx', 'vy', 'heading'],
    'absolute': True
}

# Round y-positions to the nearest lane
def laneFinder(y):
    return round(y / lane_diff)

def classifyLane(obs):
    lane_class = []
    for vehicle in obs:
        lane_class.append(laneFinder(vehicle[2]))
    return lane_class

# modified from https://github.com/eleurent/highway-env/blob/31881fbe45fd05dbd3203bb35419ff5fb1b7bc09/highway_env/vehicle/controller.py
# in this version, no extra latent state is stored (target_lane, target_speed)
TURN_HEADING = 0.15 # Target heading when turning
TURN_TARGET = 30 # How much to adjust when targeting a lane (higher = smoother)
max_velocity = 40 # Maximum velocity
turn_velocity = 30 # Turning velocity
min_velocity = 20 # Minimum velocity

last_la = {"steering": 0, "acceleration": 0 }

def run_la(self, action: Union[dict, str] = None) -> None:
    global last_la

    target_acc = 0.0
    target_heading = 0.0

    if action == "FASTER":
        # Attain max speed
        target_acc = max_velocity - self.speed

        # Follow current lane
        target_y = laneFinder(self.position[1]) * lane_diff
        target_heading = np.arctan((target_y - self.position[1]) / TURN_TARGET)
    elif action == "SLOWER":
        # Attain min speed
        target_acc = min_velocity - self.speed

        # Follow current lane
        target_y = laneFinder(self.position[1]) * lane_diff
        target_heading = np.arctan((target_y - self.position[1]) / TURN_TARGET)
    elif action == "LANE_RIGHT":
        target_acc = turn_velocity - self.speed

        # Attain rightmost heading
        target_heading = TURN_HEADING
    elif action == "LANE_LEFT":
        target_acc = turn_velocity - self.speed

        # Attain leftmost heading
        target_heading = -TURN_HEADING

    target_steer = target_heading - self.heading

    if target_steer > last_la["steering"]:
        target_steer = min(target_steer, last_la["steering"] + 0.04)
    else:
        target_steer = max(target_steer, last_la["steering"] - 0.04)

    if target_acc > last_la["acceleration"]:
        target_acc = min(target_acc, last_la["acceleration"] + 4)
    else:
        target_acc = max(target_acc, last_la["acceleration"] - 6)

    la = {"steering": target_steer, "acceleration": target_acc }

    # Add error
    # la['steering'] = np.random.normal(la['steering'], steer_err)
    # la['acceleration'] = np.random.normal(la['acceleration'], acc_err)

    last_la = la
    Vehicle.act(self, last_la)

    return la

ControlledVehicle.act = run_la

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
model.learn(50000)
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