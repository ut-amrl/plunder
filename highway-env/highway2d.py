import gym
import highway_env
from matplotlib import pyplot as plt
import os
import numpy as np
import math
import random
from typing import List, Tuple, Union, Optional
import keyboard

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


# copied from https://github.com/eleurent/highway-env/blob/31881fbe45fd05dbd3203bb35419ff5fb1b7bc09/highway_env/vehicle/controller.py
# which also contains motor model
def get_la(self, action: Union[dict, str] = None) -> None:
    """
    Perform a high-level action to change the desired lane or speed.
    - If a high-level action is provided, update the target speed and lane;
    - then, perform longitudinal and lateral control.
    :param action: a high-level action
    """
    self.follow_road()
    if action == "FASTER":
        self.target_speed += self.DELTA_SPEED
    elif action == "SLOWER":
        self.target_speed -= self.DELTA_SPEED
    elif action == "LANE_RIGHT":
        _from, _to, _id = self.target_lane_index
        target_lane_index = _from, _to, np.clip(_id + 1, 0, len(self.road.network.graph[_from][_to]) - 1)
        if self.road.network.get_lane(target_lane_index).is_reachable_from(self.position):
            self.target_lane_index = target_lane_index
    elif action == "LANE_LEFT":
        _from, _to, _id = self.target_lane_index
        target_lane_index = _from, _to, np.clip(_id - 1, 0, len(self.road.network.graph[_from][_to]) - 1)
        if self.road.network.get_lane(target_lane_index).is_reachable_from(self.position):
            self.target_lane_index = target_lane_index

    action = {"steering": self.steering_control(self.target_lane_index),
                "acceleration": self.speed_control(self.target_speed)}
    action['steering'] = np.clip(action['steering'], -self.MAX_STEERING_ANGLE, self.MAX_STEERING_ANGLE)
    return action



######## Simulation ########
ha = env.action_type.actions_indexes["IDLE"]
obs_out = open("obs_out.csv", "w")
obs_out.write("l_present, l_x, l_y, l_vx, l_vy, f_present, f_x, f_y, f_vx, f_vy, r_present, r_x, r_y, r_vx, r_vy, steering, acc, HA\n")



for _ in range(1000):
    obs, reward, done, truncated, info = env.step(ha)
    env.render()

    # Pre-process observations
    obs = classifyLane(obs)
    closest = closestVehicles(obs)


    # Run ASP
    ha = prob_asp(obs[0], closest)
    la = get_la(env.vehicle, ha)

    for v in closest:
        for prop in v:
            obs_out.write(str(prop)+", ")
    obs_out.write(str(la['steering'])+", ")
    obs_out.write(str(la['acceleration'])+", ")
    obs_out.write(str(ha))
    obs_out.write("\n")

obs_out.close()
