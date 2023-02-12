import gym
from highway_env import utils
from highway_env.envs import MDPVehicle, ControlledVehicle, Vehicle, MergeEnv
from highway_env.envs.common.observation import KinematicObservation
from matplotlib import pyplot as plt
import os
import numpy as np
import math
import random
import pprint
from typing import List, Tuple, Union, Optional

######## Env ########
num_vehicles = 3
def make_vehicles(self) -> None:
        """
        Populate a road with several vehicles on the highway and on the merging lane, as well as an ego-vehicle.

        :return: the ego-vehicle
        """
        road = self.road
        ego_vehicle = self.action_type.vehicle_class(road,
                                                     road.network.get_lane(("a", "b", 1)).position(30, 0),
                                                     speed=18)
        road.vehicles.append(ego_vehicle)

        other_vehicles_type = utils.class_from_path(self.config["other_vehicles_type"])
        for _ in range(num_vehicles):
            speed = random.randint(20, 22)
            position = random.randint(0, 200)
            road.vehicles.append(other_vehicles_type(road, road.network.get_lane(("a", "b", 0)).position(position, 0), speed=speed))

        merging_v = other_vehicles_type(road, road.network.get_lane(("j", "k", 0)).position(100, 0), speed=0)
        merging_v.target_speed = 15
        road.vehicles.append(merging_v)
        self.vehicle = ego_vehicle

MergeEnv._make_vehicles = make_vehicles


######## Configuration ########
lane_diff = 4 # Distance lanes are apart from each other
use_absolute_lanes = True # Whether or not to label lanes as absolute or relative to current vehicle lane
KinematicObservation.normalize_obs = lambda self, df: df # Don't normalize values

steer_err = 0.03
acc_err = 1.5

env = gym.make("merge-v0")
# Observations
# ego vehicle:      presence, x, y, vx, vy, heading
# 9 other vehicles: presence, x, y, vx, vy, heading        (x and y relative to ego)
env.config['observation']={
    'type': 'Kinematics',
    'vehicles_count': 10,
    'features': ['presence', 'x', 'y', 'vx', 'vy', 'heading'],
    'absolute': False
}
env.config['simulation_frequency'] = 30
env.config['policy_frequency'] = 5

ACTIONS_ALL = { # A mapping of action indexes to labels
    0: 'LANE_LEFT',
    1: 'IDLE',
    2: 'LANE_RIGHT',
    3: 'FASTER',
    4: 'SLOWER'
}

ACTION_REORDER = { # highway-env uses a different order for actions (desired: IDLE, LANE_LEFT, LANE_RIGHT)
    0: 1,
    1: 0,
    2: 2,
    3: -1, 
    4: -1
}

### Helper functions ###

# Probabilistic functions
def logistic(offset, slope, x):
    return 1.0/(1.0+np.exp(-slope*(x-offset)))

def sample(p):
    return random.random()<p

# Round y-positions to the nearest lane
def laneFinder(y):
    return round(y / lane_diff)

def closest_left(obs):
    left = [0, 100, -lane_diff, 0, 0, 0] # No car found
    for vehicle in obs:
        if vehicle[0] == 1 and laneFinder(vehicle[2]) < 0 and vehicle[1] < left[1]:
            left = vehicle.copy()
    return left

def closest_right(obs):
    right = [0, 100, lane_diff, 0, 0, 0] # No car found
    for vehicle in obs:
        if vehicle[0] == 1 and laneFinder(vehicle[2]) > 0 and vehicle[1] < right[1]:
            right = vehicle.copy()
    return right

def closest_front(obs):
    front = [0, 100, 0, 0, 0, 0] # No car found
    for vehicle in obs:
        if vehicle[0] == 1 and laneFinder(vehicle[2]) == 0 and vehicle[1] < front[1]:
            front = vehicle.copy()
    return front

### Motor model ###

# modified from https://github.com/eleurent/highway-env/blob/31881fbe45fd05dbd3203bb35419ff5fb1b7bc09/highway_env/vehicle/controller.py
# in this version, no extra latent state is stored (target_speed)
KP_H = 0.4 # Turning rate
TURN_HEADING = 0.2 # Target heading when turning
TURN_TARGET = 30 # How much to adjust when targeting a lane (higher = smoother)

last_action = "IDLE"
def run_la(self, action: Union[dict, str] = None, step = True) -> None:
    global last_action

    if action == None:
        action = last_action
    last_action = action

    target_heading = 0.0
    if action == "IDLE":
        # Follow current lane
        target_y = laneFinder(self.position[1]) * lane_diff
        target_heading = np.arctan((target_y - self.position[1]) / TURN_TARGET)
    elif action == "LANE_RIGHT":
        # Attain rightmost heading
        target_heading = TURN_HEADING
    elif action == "LANE_LEFT":
        # Attain leftmost heading
        target_heading = -TURN_HEADING

    la = {"steering": (target_heading - self.heading) * KP_H, "acceleration": 0 }

    # Add error
    la['steering'] = np.random.normal(la['steering'], steer_err)
    la['acceleration'] = np.random.normal(la['acceleration'], acc_err)

    if step: # Perform action
        Vehicle.act(self, la)
    
    return la

ControlledVehicle.act = run_la

### ASP ###

# ASP
def prob_asp(ego, left, front, right):

    # Probabilistic
    in_left_lane = sample(logistic(2, -10, ego[2]))
    right_clear = sample(logistic(20, 0.7, right[1]))
    left_clear = sample(logistic(20, 0.7, left[1]))

    if in_left_lane:
        if right_clear:
            return env.action_type.actions_indexes["LANE_RIGHT"]
        else:
            return env.action_type.actions_indexes["IDLE"]
    else:
        if right_clear:
            return env.action_type.actions_indexes["IDLE"]
        elif left_clear:
            return env.action_type.actions_indexes["LANE_LEFT"]
    return env.action_type.actions_indexes["IDLE"]

def runSim(iter):
    env.reset()
    ha = env.action_type.actions_indexes["IDLE"]
    obs_out = open("data" + str(iter) + ".csv", "w")
    obs_out.write("x, y, vx, vy, heading, l_x, l_y, l_vx, l_vy, l_heading, f_x, f_y, f_vx, f_vy, f_heading, r_x, r_y, r_vx, r_vy, r_heading, LA.steer, LA.acc, HA\n")

    for _ in range(120):
        obs, reward, done, truncated, info = env.step(ha)
        env.render()

        left = closest_left(obs[1:])
        front = closest_front(obs[1:])
        right = closest_right(obs[1:])

        # Run ASP
        ha = prob_asp(obs[0], left, front, right)

        # Run motor model
        la = run_la(env.vehicle, ACTIONS_ALL[ha], False)

        def print_vehicle(vehicle):
            for prop in vehicle[1:]:
                obs_out.write(str(round(prop, 3)) +", ")

        print_vehicle(obs[0])
        print_vehicle(left)
        print_vehicle(front)
        print_vehicle(right)
        
        obs_out.write(str(round(la['steering'], 3))+", ")
        obs_out.write(str(round(la['acceleration'], 3))+", ")
        obs_out.write(str(ACTION_REORDER[ha]))
        obs_out.write("\n")

    obs_out.close()

for iter in range(12):
    runSim(iter)

num_vehicles = 2
for iter in range(12, 14):
    runSim(iter)

num_vehicles = 4
for iter in range(14, 17):
    runSim(iter)

num_vehicles = 6
for iter in range(17, 20):
    runSim(iter)