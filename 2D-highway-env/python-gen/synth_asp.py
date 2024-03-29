import gymnasium as gym
import highway_env
from highway_env.envs import ControlledVehicle, Vehicle
from highway_env.envs.common.observation import KinematicObservation
import numpy as np
import random
from typing import Union

env = gym.make('highway-fast-v0')

######## Configuration ########
lane_diff = 4 # Distance lanes are apart from each other
lanes_count = 8 # Number of lanes
use_absolute_lanes = True # Whether or not to label lanes as absolute or relative to current vehicle lane
KinematicObservation.normalize_obs = lambda self, df: df # Don't normalize values

steer_err = 0.01
acc_err = 1

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

def logistic2(x, offset, slope):
    return 1.0/(1.0+np.exp(-slope*(x-offset)))

def sample(p):
    return random.random()<p

def Plus(x, y):
    return x + y

def Abs(x):
    return abs(x)

def Minus(x, y):
    return x - y

def DividedBy(x, y):
    return x / y

def And(x, y):
    return x and y

def Or(x, y):
    return x or y

def Lt(x, y):
    return x < y

def Gt(x, y):
    return x > y

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
def gt(ego, closest, ha):
    x, l_x, f_x, r_x = ego[1], closest[0][1], closest[1][1], closest[2][1] 
    l_x, f_x, r_x = l_x - x, f_x - x, r_x - x
    vx = ego[3]

    if ha == env.action_type.actions_indexes["FASTER"]:
        front_clear = sample(logistic(1, 30, f_x / vx))
    else:
        front_clear = sample(logistic(1.5, 30, f_x / vx))
    left_clear = sample(logistic(1, 30, l_x / vx))
    right_clear = sample(logistic(1, 30, r_x / vx))
    left_better = sample(logistic(0, 1, l_x - r_x))

    if front_clear: # No car in front: accelerate
        return env.action_type.actions_indexes["FASTER"]
    if not ha == env.action_type.actions_indexes["LANE_RIGHT"] and left_clear and left_better: # No car on the left: merge left
        return env.action_type.actions_indexes["LANE_LEFT"]
    if not ha == env.action_type.actions_indexes["LANE_LEFT"] and right_clear: # No car on the right: merge right
        return env.action_type.actions_indexes["LANE_RIGHT"]

    # Nowhere to go: decelerate
    return env.action_type.actions_indexes["SLOWER"]

def plunder(ego, closest, ha):
    x, l_x, f_x, r_x = ego[1], closest[0][1], closest[1][1], closest[2][1] 
    vx = ego[3]
    f_vx = closest[1][3]
    r_vx = closest[2][3]

    if ha == env.action_type.actions_indexes["FASTER"] and And(sample(logistic2(Minus(r_x, l_x), -42.325607, -11.322358)), sample(logistic2(Minus(x, f_x), -39.853004, 0.471738))):
        return env.action_type.actions_indexes["LANE_LEFT"]
    if ha == env.action_type.actions_indexes["FASTER"] and And(sample(logistic2(Minus(r_x, f_x), 2.268923, 1.691306)), sample(logistic2(Minus(x, f_x), -37.500061, 0.498594))):
        return env.action_type.actions_indexes["LANE_RIGHT"]
    if ha == env.action_type.actions_indexes["FASTER"] and sample(logistic2(DividedBy(Minus(f_x, x), vx), 0.977072, -31.665253)):
        return env.action_type.actions_indexes["SLOWER"]
    if ha == env.action_type.actions_indexes["LANE_LEFT"] and sample(logistic2(DividedBy(Minus(f_x, x), r_vx), 2.090207, 93.243362)):
        return env.action_type.actions_indexes["FASTER"]
    if ha == env.action_type.actions_indexes["LANE_LEFT"] and sample(logistic2(f_vx, 1000000000.000000, 1.000000)):
        return env.action_type.actions_indexes["LANE_RIGHT"]
    if ha == env.action_type.actions_indexes["LANE_LEFT"] and sample(logistic2(Minus(l_x, x), 30.264160, -3.164596)):
        return env.action_type.actions_indexes["SLOWER"]
    if ha == env.action_type.actions_indexes["LANE_RIGHT"] and sample(logistic2(DividedBy(Minus(f_x, x), vx), 1.538538, 14.793274)):
        return env.action_type.actions_indexes["FASTER"]
    if ha == env.action_type.actions_indexes["LANE_RIGHT"] and sample(logistic2(r_vx, 1000000000.000000, 1.000000)):
        return env.action_type.actions_indexes["LANE_LEFT"]
    if ha == env.action_type.actions_indexes["LANE_RIGHT"] and sample(logistic2(DividedBy(Minus(x, r_x), vx), -0.984751, 182.378754)):
        return env.action_type.actions_indexes["SLOWER"]
    if ha == env.action_type.actions_indexes["SLOWER"] and sample(logistic2(DividedBy(Minus(f_x, x), vx), 1.495547, 73.774384)):
        return env.action_type.actions_indexes["FASTER"]
    if ha == env.action_type.actions_indexes["SLOWER"] and sample(logistic2(Minus(l_x, f_x), 5.026824, 4.910437)):
        return env.action_type.actions_indexes["LANE_LEFT"]
    if ha == env.action_type.actions_indexes["SLOWER"] and sample(logistic2(Minus(r_x, f_x), 1.479489, 1.003578)):
        return env.action_type.actions_indexes["LANE_RIGHT"]
    return ha

def oneshot(ego, closest, ha):
    x, l_x, f_x, r_x = ego[1], closest[0][1], closest[1][1], closest[2][1] 
    vx = ego[3]
    f_vx = closest[1][3]
    l_vx = closest[0][3]
    r_vx = closest[2][3]

    if ha == env.action_type.actions_indexes["FASTER"] and And(sample(logistic2(Minus(l_x, r_x), 44.971569, 0.093222)), sample(logistic2(Minus(x, f_x), -39.456978, 0.540623))):
        return env.action_type.actions_indexes["LANE_LEFT"]
    if ha == env.action_type.actions_indexes["FASTER"] and sample(logistic2(Minus(f_x, x), 24.905714, -0.193740)):
        return env.action_type.actions_indexes["LANE_RIGHT"]
    if ha == env.action_type.actions_indexes["FASTER"] and sample(logistic2(Minus(f_x, x), 31.530993, -0.293225)):
        return env.action_type.actions_indexes["SLOWER"]
    if ha == env.action_type.actions_indexes["LANE_LEFT"] and And(sample(logistic2(Minus(x, f_x), -47.838818, -0.157153)), sample(logistic2(l_vx, 14.125803, 0.222442))):
        return env.action_type.actions_indexes["FASTER"]
    if ha == env.action_type.actions_indexes["LANE_LEFT"] and sample(logistic2(Minus(f_x, x), 117.364967, 0.056507)):
        return env.action_type.actions_indexes["LANE_RIGHT"]
    if ha == env.action_type.actions_indexes["LANE_LEFT"] and sample(logistic2(Minus(x, l_x), -18.254396, 0.155949)):
        return env.action_type.actions_indexes["SLOWER"]
    if ha == env.action_type.actions_indexes["LANE_RIGHT"] and sample(logistic2(Minus(x, f_x), -70.778740, -0.065321)):
        return env.action_type.actions_indexes["FASTER"]
    if ha == env.action_type.actions_indexes["LANE_RIGHT"] and sample(logistic2(r_vx, 34.945477, 0.281471)):
        return env.action_type.actions_indexes["LANE_LEFT"]
    if ha == env.action_type.actions_indexes["LANE_RIGHT"] and And(sample(logistic2(Minus(x, r_x), -16.835064, 0.121348)), sample(logistic2(x, 396.793243, 3.467931))):
        return env.action_type.actions_indexes["SLOWER"]
    if ha == env.action_type.actions_indexes["SLOWER"] and sample(logistic2(Minus(f_x, x), 55.042988, 0.181467)):
        return env.action_type.actions_indexes["FASTER"]
    if ha == env.action_type.actions_indexes["SLOWER"] and And(sample(logistic2(Minus(l_x, x), 35.048420, 0.505908)), sample(logistic2(vx, 38.985645, -2.734211))):
        return env.action_type.actions_indexes["LANE_LEFT"]
    if ha == env.action_type.actions_indexes["SLOWER"] and sample(logistic2(Minus(f_x, r_x), -23.988991, -0.110289)):
        return env.action_type.actions_indexes["LANE_RIGHT"]
    return ha

def greedy(ego, closest, ha):
    x, l_x, f_x, r_x = ego[1], closest[0][1], closest[1][1], closest[2][1] 
    vx = ego[3]
    f_vx = closest[1][3]
    l_vx = closest[0][3]
    r_vx = closest[2][3]

    if ha == env.action_type.actions_indexes["FASTER"] and sample(logistic2(Minus(f_x, x), -3.151449, -0.058887)):
        return env.action_type.actions_indexes["LANE_LEFT"]
    if ha == env.action_type.actions_indexes["FASTER"] and sample(logistic2(Minus(x, f_x), -9.845424, 0.068582)):
        return env.action_type.actions_indexes["LANE_RIGHT"]
    if ha == env.action_type.actions_indexes["FASTER"] and sample(logistic2(vx, 16.533262, -0.091292)):
        return env.action_type.actions_indexes["SLOWER"]
    if ha == env.action_type.actions_indexes["LANE_LEFT"] and And(sample(logistic2(l_vx, 14.960572, 0.323381)), sample(logistic2(Minus(x, f_x), -43.630104, -0.126041))):
        return env.action_type.actions_indexes["FASTER"]
    if ha == env.action_type.actions_indexes["LANE_LEFT"] and sample(logistic2(Minus(r_x, l_x), 29.015543, 0.018440)):
        return env.action_type.actions_indexes["LANE_RIGHT"]
    if ha == env.action_type.actions_indexes["LANE_LEFT"] and sample(logistic2(Minus(l_x, x), 21.814777, -0.059201)):
        return env.action_type.actions_indexes["SLOWER"]
    if ha == env.action_type.actions_indexes["LANE_RIGHT"] and And(sample(logistic2(l_vx, 14.128410, 0.216316)), sample(logistic2(Minus(x, f_x), -42.581684, -0.184605))):
        return env.action_type.actions_indexes["FASTER"]
    if ha == env.action_type.actions_indexes["LANE_RIGHT"] and sample(logistic2(r_vx, 32.813919, 0.115039)):
        return env.action_type.actions_indexes["LANE_LEFT"]
    if ha == env.action_type.actions_indexes["LANE_RIGHT"] and sample(logistic2(Minus(r_x, x), 24.849457, -0.061592)):
        return env.action_type.actions_indexes["SLOWER"]
    if ha == env.action_type.actions_indexes["SLOWER"] and sample(logistic2(Minus(x, f_x), -41.966869, -0.099316)):
        return env.action_type.actions_indexes["FASTER"]
    if ha == env.action_type.actions_indexes["SLOWER"] and And(sample(logistic2(vx, 26.373911, 0.768933)), sample(logistic2(Minus(l_x, x), 31.935848, 0.080790))):
        return env.action_type.actions_indexes["LANE_LEFT"]
    if ha == env.action_type.actions_indexes["SLOWER"] and sample(logistic2(Minus(r_x, f_x), 7.489878, 0.092010)):
        return env.action_type.actions_indexes["LANE_RIGHT"]
    return ha

def ldips(ego, closest, ha):
    x, l_x, f_x, r_x = ego[1], closest[0][1], closest[1][1], closest[2][1] 
    vx = ego[3]
    f_vx = closest[1][3]
    l_vx = closest[0][3]
    r_vx = closest[2][3]

    if ha == env.action_type.actions_indexes["FASTER"] and And(Gt(l_x, 800.216980), Gt(Minus(x, f_x), -35.705017)):
        return env.action_type.actions_indexes["LANE_LEFT"]
    if ha == env.action_type.actions_indexes["FASTER"] and And(Gt(Minus(x, f_x), -23.054993), Gt(vx, 23.037001)):
        return env.action_type.actions_indexes["LANE_RIGHT"]
    if ha == env.action_type.actions_indexes["FASTER"] and And(Gt(Minus(x, f_x), -36.483002), Gt(vx, 36.361000)):
        return env.action_type.actions_indexes["SLOWER"]
    if ha == env.action_type.actions_indexes["LANE_LEFT"] and And(Lt(Minus(x, f_x), -53.286987), Gt(x, 200.119995)):
        return env.action_type.actions_indexes["FASTER"]
    if ha == env.action_type.actions_indexes["LANE_LEFT"] and And(Lt(Minus(x, r_x), -58.799988), Gt(x, 424.480011)):
        return env.action_type.actions_indexes["LANE_RIGHT"]
    if ha == env.action_type.actions_indexes["LANE_LEFT"] and And(Gt(Minus(x, l_x), -34.805012), Gt(vx, 35.834999)):
        return env.action_type.actions_indexes["SLOWER"]
    if ha == env.action_type.actions_indexes["LANE_RIGHT"] and Or(Lt(Minus(x, f_x), -51.644958), Lt(vx, 22.355000)):
        return env.action_type.actions_indexes["FASTER"]
    if ha == env.action_type.actions_indexes["LANE_RIGHT"] and And(Lt(r_vx, 19.618000), Gt(Plus(x, x), 814.185059)):
        return env.action_type.actions_indexes["LANE_LEFT"]
    if ha == env.action_type.actions_indexes["LANE_RIGHT"] and And(Gt(x, 519.620972), Gt(Minus(x, r_x), -31.039000)):
        return env.action_type.actions_indexes["SLOWER"]
    if ha == env.action_type.actions_indexes["SLOWER"] and Or(Lt(l_x, 259.065002), Lt(Minus(x, f_x), -50.816956)):
        return env.action_type.actions_indexes["FASTER"]
    if ha == env.action_type.actions_indexes["SLOWER"] and Gt(Minus(l_x, x), 36.622009):
        return env.action_type.actions_indexes["LANE_LEFT"]
    if ha == env.action_type.actions_indexes["SLOWER"] and Gt(Minus(r_x, f_x), 1.148010):
        return env.action_type.actions_indexes["LANE_RIGHT"]
    return ha

# modified from https://github.com/eleurent/highway-env/blob/31881fbe45fd05dbd3203bb35419ff5fb1b7bc09/highway_env/vehicle/controller.py
# in this version, no extra latent state is stored (target_lane, target_speed)
TURN_HEADING = 0.15 # Target heading when turning
TURN_TARGET = 30 # How much to adjust when targeting a lane (higher = smoother)
max_velocity = 40 # Maximum velocity
turn_velocity = 30 # Turning velocity

last_action = "FASTER"
last_target = 0
last_la = {"steering": 0, "acceleration": 0 }

def run_la(self, action: Union[dict, str] = None, step = True, closest = None) -> None:
    global last_action, last_la, last_target

    if step:
        Vehicle.act(self, last_la)
        return last_la

    target_acc = 0.0
    target_heading = 0.0

    if action == None:
        action = last_action
        
    last_action = action

    if action == "FASTER":
        # Attain max speed
        target_acc = max_velocity - self.speed

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
        target_acc = (last_target - self.speed)

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
    la['steering'] = np.random.normal(la['steering'], steer_err)
    la['acceleration'] = np.random.normal(la['acceleration'], acc_err)

    if not closest == None:
        last_la = la
    
    return la

ControlledVehicle.act = run_la

######## Simulation ########
success, dist = 0, 0
def runSim(iter):
    global success, dist
    env.reset()
    ha = env.action_type.actions_indexes["FASTER"]

    start = -1
    for t_step in range(150):
        obs, reward, done, truncated, info = env.step(ha)
        env.render()

        # Pre-process observations
        lane_class = classifyLane(obs)
        closest = closestVehicles(obs, lane_class)

        # Run ASP
        ha = gt(obs[0], closest, ha)
        # Run motor model
        la = run_la(env.vehicle, ACTIONS_ALL[ha], False, closest)

        if start < 0:
            start = obs[0][1]

    if(obs[0][3] > 10 and laneFinder(obs[0][2]) >= 0 and laneFinder(obs[0][2]) <= 7): # Positive velocity and in an existing lane
        success += 1
    dist += obs[0][1] - start

for iter in range(200):
    runSim(iter)

print(success)
print(dist / 200)