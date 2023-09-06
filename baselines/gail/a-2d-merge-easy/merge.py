import gymnasium
from highway_env.envs import MDPVehicle, ControlledVehicle, Vehicle, highway_env
from highway_env.envs.common import graphics
from highway_env.utils import near_split, class_from_path
from matplotlib import pyplot as plt
import numpy as np
import random
from typing import List, Tuple, Union, Optional


######## Configuration ########
lane_diff = 4 # Distance lanes are apart from each other
lanes_count = 6 # Number of lanes

steer_err = 0.005
acc_err = 0.5


ACTIONS_ALL = { # A mapping of action indexes to labels
    0: 'LANE_LEFT',
    1: 'IDLE',
    2: 'LANE_RIGHT',
    3: 'FASTER',
    4: 'SLOWER'
}

ACTION_REORDER = { # highway-env uses a different order for actions (desired: FASTER, SLOWER, LANE_LEFT, LANE_RIGHT)
    0: 2,
    1: -1,
    2: 3,
    3: 0, 
    4: 1
}

######## ASP ########
# Probabilistic functions
def logistic(offset, slope, x):
    return 1.0/(1.0+np.exp(-slope*(x-offset)))

def sample(p):
    return random.random()<p

# Helper functions

# Round y-positions to the nearest lane
def laneFinder(y):
    return round(y / lane_diff)

def classifyLane(obs):
    lane_class = []
    for vehicle in obs:
        lane_class.append(laneFinder(vehicle[2]))
    return lane_class

# Find closest vehicles in lanes next to the ego vehicle
# Assumption: vehicles are already sorted based on x distance (ignores vehicles behind the ego vehicle)
def closestInLane(obs, lane, lane_class, ego):
    for i in range(len(obs)):
        if obs[i][0] == 0: # not present
            continue
        if lane_class[i] == lane: # in desired lane
            return obs[i]
    
    return [0, ego[1] + 100, lane * lane_diff, ego[3], ego[4], ego[5]] # No car found

def closestVehicles(obs, lane_class):
    ego_lane = laneFinder(obs[0][2])

    closestLeft = closestInLane(obs[1:], ego_lane - 1, lane_class[1:], obs[0])
    closestFront = closestInLane(obs[1:], ego_lane, lane_class[1:], obs[0])
    closestRight = closestInLane(obs[1:], ego_lane + 1, lane_class[1:], obs[0])

    # Handle edges (in rightmost or leftmost lane)
    if lane_class[0] == 0: # In leftmost lane: pretend there is a vehicle to the left
        closestLeft = obs[0].copy()
        closestLeft[2] = obs[0][2] - lane_diff
    if lane_class[0] == env.config['lanes_count'] - 1: # In rightmost lane: pretend there is a vehicle to the right
        closestRight = obs[0].copy()
        closestRight[2] = obs[0][2] + lane_diff
    
    return (closestLeft, closestFront, closestRight)

# ASP (probabilistic)
def prob_asp(ego, closest, ha):
    x, l_x, f_x, r_x = ego[1], closest[0][1], closest[1][1], closest[2][1]
    vx = ego[3]

    front_clear = sample(logistic(1, 40, (f_x - x) / vx))

    if ha == env.action_type.actions_indexes["LANE_RIGHT"]:
        right_clear = sample(logistic(0.5, 40, (r_x - x) / vx)) # time to collision
        if right_clear: # No car on the right: merge right
            return env.action_type.actions_indexes["LANE_RIGHT"]
        if front_clear: 
            return env.action_type.actions_indexes["FASTER"]

        # Nowhere to go: decelerate
        return env.action_type.actions_indexes["SLOWER"]

    right_clear = sample(logistic(1, 40, (r_x - x) / vx)) # time to collision
    if right_clear: # No car on the right: merge right
        return env.action_type.actions_indexes["LANE_RIGHT"]
    if front_clear: 
        return env.action_type.actions_indexes["FASTER"]

    # Nowhere to go: decelerate
    return env.action_type.actions_indexes["SLOWER"]

# modified from https://github.com/eleurent/highway-env/blob/31881fbe45fd05dbd3203bb35419ff5fb1b7bc09/highway_env/vehicle/controller.py
# in this version, no extra latent state is stored (target_lane, target_speed)
TURN_HEADING = 0.15 # Target heading when turning
TURN_TARGET = 30 # How much to adjust when targeting a lane (higher = smoother)
max_velocity = 45 # Maximum velocity

last_action = "FASTER"
last_target = 0
last_la = {"steering": 0, "acceleration": 0 }



######## Simulation ########
def runSim(iter):
    env.reset()
    graphics.LAST_HA = "FASTER"
    ha = env.action_type.actions_indexes[graphics.LAST_HA]
    obs_out = open("data" + str(iter) + ".csv", "w")
    obs_out.write("x, y, vx, vy, heading, l_x, l_y, l_vx, l_vy, l_heading, f_x, f_y, f_vx, f_vy, f_heading, r_x, r_y, r_vx, r_vy, r_heading, LA.steer, LA.acc, HA\n")

    for _ in range(75):

        obs, reward, done, truncated, info = env.step(ha)
        env.render()

        # Pre-process observations

        # # Run ASP
        # ha = env.action_type.actions_indexes[graphics.LAST_HA]
        ha = prob_asp(obs[0], closest, ha)

        # Run motor model
        la = run_la(env.vehicle, ACTIONS_ALL[ha], False, closest)

        # Ego vehicle
        for prop in obs[0][1:]:
            obs_out.write(str(round(prop, 3))+", ")
        
        # Nearby vehicles
        for v in closest:
            for prop in v[1:]:
                obs_out.write(str(round(prop, 3))+", ")
        obs_out.write(str(round(la['steering'], 3))+", ")
        obs_out.write(str(round(la['acceleration'], 3))+", ")
        obs_out.write(str(ACTION_REORDER[ha]))
        obs_out.write("\n")


    obs_out.close()

# for iter in range(25):
#     runSim(iter)
# iter = 25
# Run generalized simulations involving more vehicles, lanes, etc.

# env.config['lanes_count']=lanes_count+2
# env.config['observation']={
#     'type': 'Kinematics',
#     'vehicles_count': 50,
#     'features': ['presence', 'x', 'y', 'vx', 'vy', 'heading'],
#     'absolute': True
# }
# runSim(iter)
# iter += 1
# runSim(iter)
# iter += 1
# runSim(iter)
# iter += 1
# runSim(iter)
# iter += 1
# runSim(iter)
# iter += 1