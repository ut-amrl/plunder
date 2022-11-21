import gym
import highway_env
from matplotlib import pyplot as plt
import os
import numpy as np
import math
import random

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

highway_env.highway_env.envs.ControlledVehicle.DELTA_SPEED = 50 # Acceleration / Deceleration
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

######## Simulation ########
action = env.action_type.actions_indexes["IDLE"]
for _ in range(1000):
    obs, reward, done, truncated, info = env.step(action)
    env.render()

    # Pre-process observations
    obs = classifyLane(obs)
    closest = closestVehicles(obs)

    # Run ASP
    action = prob_asp(obs[0], closest)
