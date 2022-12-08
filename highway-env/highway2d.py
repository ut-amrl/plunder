import gym
import highway_env
from matplotlib import pyplot as plt
import os
import numpy as np
import math
import random
from typing import List, Tuple, Union, Optional

######## Configuration ########
lane_diff = 0.25 # Distance lanes are apart from each other
lanes_count = 4 # Number of lanes

env = gym.make('highway-v0')
env.config['simulation_frequency']=20
env.config['policy_frequency']=2 # Runs once every 10 simulation steps
env.config['lanes_count']=lanes_count

# Observations
# ego vehicle:      presence, x, y, vx, vy
# 9 other vehicles: presence, x, y, vx, vy        (relative to ego)
env.config['observation']={
    'type': 'Kinematics',
    'vehicles_count': 10,
    'features': ['presence', 'x', 'y', 'vx', 'vy'],
    'absolute': False
}

ACTIONS_ALL = { # A mapping of action indexes to labels
    0: 'LANE_LEFT',
    1: 'IDLE',
    2: 'LANE_RIGHT',
    3: 'FASTER',
    4: 'SLOWER'
}

highway_env.highway_env.envs.MDPVehicle.DEFAULT_TARGET_SPEEDS = np.linspace(20, 30, 5) # Speed bounds
highway_env.highway_env.envs.ControlledVehicle.DELTA_SPEED = 5 # Acceleration / Deceleration

env.reset()

######## ASP ########
# Probabilistic functions
def logistic(slope, offset, x):
    return 1.0/(1.0+np.exp(-slope*(x-offset)))

def sample(p):
    return random.random()<p



# Helper functions

# Round y-positions to the nearest lane
def classifyLane(obs):
    for vehicle in obs:
        vehicle[2] = round(vehicle[2] / lane_diff)
    return obs

# Find closest vehicles in lanes next to the ego vehicle
# Assumption: vehicles are already sorted based on x distance (ignores vehicles behind the ego vehicle)
def closestInLane(obs, lane):
    for vehicle in obs:
        if vehicle[0] == 0: # not present
            continue
        if vehicle[2] == lane: # in desired lane
            return vehicle
    
    return [0, 1000000000, lane, 0, 0] # No car found

def closestVehicles(obs):
    closestLeft = closestInLane(obs[1:], -1)
    closestFront = closestInLane(obs[1:], 0)
    closestRight = closestInLane(obs[1:], 1)

    # Handle edges (in rightmost or leftmost lane)
    if obs[0][2] == 0: # In leftmost lane: pretend there is a vehicle to the left
        closestLeft = obs[0]
        closestLeft[1] = 0
        closestLeft[2] = -1
    if obs[0][2] == lanes_count - 1: # In rightmost lane: pretend there is a vehicle to the right
        closestRight = obs[0]
        closestRight[1] = 0
        closestRight[2] = 1
    
    return (closestLeft, closestFront, closestRight)



# ASP (deterministic)
def det_asp(ego, closest):
    car_in_front = closest[1][1] < 0.1
    car_left = closest[0][1] < 0.1
    car_right = closest[2][1] < 0.1

    if not car_in_front: # No car in front: accelerate
        return env.action_type.actions_indexes["FASTER"]
    if not car_left: # No car on the left: merge left
        return env.action_type.actions_indexes["LANE_LEFT"]
    if not car_right: # No car on the right: merge right
        return env.action_type.actions_indexes["LANE_RIGHT"]

    # Nowhere to go: decelerate
    return env.action_type.actions_indexes["SLOWER"]

# ASP (probabilistic)
def prob_asp(ego, closest):
    car_in_front = sample(logistic(-50, 0.15, closest[1][1]))
    car_left = sample(logistic(-50, 0.15, closest[0][1]))
    car_right = sample(logistic(-50, 0.15, closest[2][1]))

    if not car_in_front: # No car in front: accelerate
        return env.action_type.actions_indexes["FASTER"]
    if not car_left: # No car on the left: merge left
        return env.action_type.actions_indexes["LANE_LEFT"]
    if not car_right: # No car on the right: merge right
        return env.action_type.actions_indexes["LANE_RIGHT"]

    # Nowhere to go: decelerate
    return env.action_type.actions_indexes["SLOWER"]

# copied from https://github.com/eleurent/highway-env/blob/31881fbe45fd05dbd3203bb35419ff5fb1b7bc09/highway_env/vehicle/controller.py
# which also contains motor model
def get_la(self, action: Union[dict, str] = None) -> None:
    la = {"steering": self.steering_control(self.target_lane_index),
            "acceleration": self.speed_control(self.target_speed)}
    return la



######## Simulation ########
ha = env.action_type.actions_indexes["FASTER"]
obs_out = open("obs_out.csv", "w")
obs_out.write("left_present, l_x, l_y, l_vx, l_vy, forward_present, f_x, f_y, f_vx, f_vy, right_present, r_x, r_y, r_vx, r_vy, steering, acc, HA\n")



for _ in range(1000):
    obs, reward, done, truncated, info = env.step(ha)
    env.render()

    # Pre-process observations
    obs = classifyLane(obs)
    closest = closestVehicles(obs)

    # Run motor model
    la = get_la(env.vehicle, ha)

    for v in closest:
        for prop in v:
            obs_out.write(str(prop)+", ")
    obs_out.write(str(la['steering'])+", ")
    obs_out.write(str(la['acceleration'])+", ")
    obs_out.write(ACTIONS_ALL[ha])
    obs_out.write("\n")
    
    # Run ASP
    ha = prob_asp(env.vehicle, closest)

obs_out.close()
