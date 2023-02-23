import gym
from highway_env.envs import MDPVehicle, ControlledVehicle, Vehicle
from highway_env.envs.common.observation import KinematicObservation
from matplotlib import pyplot as plt
import numpy as np
import random
from typing import List, Tuple, Union, Optional

######## Configuration ########
lane_diff = 4 # Distance lanes are apart from each other
lanes_count = 4 # Number of lanes
use_absolute_lanes = True # Whether or not to label lanes as absolute or relative to current vehicle lane
KinematicObservation.normalize_obs = lambda self, df: df # Don't normalize values

steer_err = 0.01
acc_err = 1

env = gym.make('highway-v0')
env.config['simulation_frequency']=16
env.config['policy_frequency']=2 # Runs once every 8 simulation steps
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
    
    return [0, 100, lane * lane_diff, ego[3], ego[4], ego[5]] # No car found

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
def prob_asp(ego, closest):
    front_clear = sample(logistic(30, 1, closest[1][1] - ego[1]))
    left_clear = sample(logistic(30, 1, closest[0][1] - ego[1]))
    right_clear = sample(logistic(30, 1, closest[2][1] - ego[1]))
    left_better = sample(logistic(0, 1, closest[0][1] - closest[2][1]))

    if front_clear: # No car in front: accelerate
        return env.action_type.actions_indexes["FASTER"]
    if left_clear and left_better: # No car on the left: merge left
        return env.action_type.actions_indexes["LANE_LEFT"]
    if right_clear: # No car on the right: merge right
        return env.action_type.actions_indexes["LANE_RIGHT"]

    # Nowhere to go: decelerate
    return env.action_type.actions_indexes["SLOWER"]

    # Synthesized ASP
    x, l_x, f_x, r_x = ego[1], closest[0][1], closest[1][1], closest[2][1]
    if ha == env.action_type.actions_indexes["FASTER"]:
        if sample(logistic(2.95, -1.67, r_x - l_x)) and sample(logistic(-4.92, -6.53, f_x - l_x)) and sample(logistic(-30.62, 1.78, x - f_x)):
            return env.action_type.actions_indexes["LANE_LEFT"]
        if sample(logistic(2.65, 5.98, r_x - f_x)) and sample(logistic(30.97, -1.43, f_x - x)):
            return env.action_type.actions_indexes["LANE_RIGHT"]
        if sample(logistic(29.04, -1.16, f_x - x)):
            return env.action_type.actions_indexes["SLOWER"]
    elif ha == env.action_type.actions_indexes["LANE_LEFT"]:
        if sample(logistic(30.79, 8.5, f_x - x)):
            return env.action_type.actions_indexes["FASTER"]
        if False:
            return env.action_type.actions_indexes["LANE_RIGHT"]
        if sample(logistic(-31.16, 2.09, x - l_x)):
            return env.action_type.actions_indexes["SLOWER"]
    elif ha == env.action_type.actions_indexes["LANE_RIGHT"]:
        if sample(logistic(52.85, 1.38, closest[0][3] + closest[0][3])) or sample(logistic(-30.75, -6.5, x - f_x)):
            return env.action_type.actions_indexes["FASTER"]
        if sample(logistic(-4.98, -0.97, r_x - l_x)):
            return env.action_type.actions_indexes["LANE_LEFT"]
        if sample(logistic(-30.8, 11.13, x - r_x)):
            return env.action_type.actions_indexes["SLOWER"]
    elif ha == env.action_type.actions_indexes["SLOWER"]:
        if sample(logistic(-312.88, -0.006, r_x)):
            return env.action_type.actions_indexes["FASTER"]
        if sample(logistic(-42.61, -1.26, r_x - l_x)):
            return env.action_type.actions_indexes["LANE_LEFT"]
        if sample(logistic(1575.17, 0.03, r_x)) or sample(logistic(29.62, 10.37, r_x - x)):
            return env.action_type.actions_indexes["LANE_RIGHT"]

# modified from https://github.com/eleurent/highway-env/blob/31881fbe45fd05dbd3203bb35419ff5fb1b7bc09/highway_env/vehicle/controller.py
# in this version, no extra latent state is stored (target_lane, target_speed)
KP_H = 0.5 # Turning rate
TURN_HEADING = 0.15 # Target heading when turning
TURN_TARGET = 30 # How much to adjust when targeting a lane (higher = smoother)

min_velocity = 16 # Minimum velocity
max_velocity = 30 # Maximum velocity

last_action = "FASTER"
last_target = 0
def run_la(self, action: Union[dict, str] = None, step = True, closest = None) -> None:
    acc = 0.0
    target_heading = 0.0

    global last_action
    global last_target

    if action == None:
        action = last_action
        
    last_action = action

    if action == "FASTER":
        # Attain max speed
        acc = 0.4 * (max_velocity - self.speed)

        # Follow current lane
        target_y = laneFinder(self.position[1]) * lane_diff
        target_heading = np.arctan((target_y - self.position[1]) / TURN_TARGET)
    elif action == "SLOWER":
        # Attain speed of vehicle in front
        if closest == None:
            front_speed = last_target
        else:
            front_speed = closest[1][3]

        last_target = front_speed
        acc = (last_target - self.speed)

        # Follow current lane
        target_y = laneFinder(self.position[1]) * lane_diff
        target_heading = np.arctan((target_y - self.position[1]) / TURN_TARGET)
    elif action == "LANE_RIGHT":
        # Attain rightmost heading
        target_heading = TURN_HEADING
    elif action == "LANE_LEFT":
        # Attain leftmost heading
        target_heading = -TURN_HEADING

    la = {"steering": (target_heading - self.heading) * KP_H, "acceleration": acc }

    # Add error
    la['steering'] = np.random.normal(la['steering'], steer_err)
    la['acceleration'] = np.random.normal(la['acceleration'], acc_err)

    if step: # Perform action
        Vehicle.act(self, la)
    
    return la

ControlledVehicle.act = run_la

######## Simulation ########
def runSim(iter):
    env.reset()
    ha = env.action_type.actions_indexes["FASTER"]
    obs_out = open("data" + str(iter) + ".csv", "w")
    obs_out.write("x, y, vx, vy, heading, l_x, l_y, l_vx, l_vy, l_heading, f_x, f_y, f_vx, f_vy, f_heading, r_x, r_y, r_vx, r_vy, r_heading, LA.steer, LA.acc, HA\n")

    for _ in range(100):

        obs, reward, done, truncated, info = env.step(ha)
        # env.render()

        # Pre-process observations
        lane_class = classifyLane(obs)
        closest = closestVehicles(obs, lane_class)

        # Run ASP
        ha = prob_asp(obs[0], closest)

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

for iter in range(25):
    runSim(iter)
iter = 25
# Run generalized simulations involving more vehicles, lanes, etc.

env.config['lanes_count']=lanes_count+2
env.config['observation']={
    'type': 'Kinematics',
    'vehicles_count': 50,
    'features': ['presence', 'x', 'y', 'vx', 'vy', 'heading'],
    'absolute': True
}
runSim(iter)
iter += 1
runSim(iter)
iter += 1

env.config['lanes_count']=lanes_count-2
env.config['observation']={
    'type': 'Kinematics',
    'vehicles_count': 50,
    'features': ['presence', 'x', 'y', 'vx', 'vy', 'heading'],
    'absolute': True
}
runSim(iter)
iter += 1
runSim(iter)
iter += 1
runSim(iter)
iter += 1