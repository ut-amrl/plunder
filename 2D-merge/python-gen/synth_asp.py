import gymnasium as gym
import highway_env
from highway_env.envs import MDPVehicle, ControlledVehicle, Vehicle, highway_env
from highway_env.envs.common.observation import KinematicObservation
from highway_env.envs.common import graphics
from highway_env.utils import near_split, class_from_path
from matplotlib import pyplot as plt
import numpy as np
import random
from typing import List, Tuple, Union, Optional

def _create_vehicles(self) -> None:
        """Create some new random vehicles of a given type, and add them on the road."""
        other_vehicles_type = class_from_path(self.config["other_vehicles_type"])
        other_per_controlled = near_split(self.config["vehicles_count"], num_bins=self.config["controlled_vehicles"])

        self.controlled_vehicles = []
        for others in other_per_controlled:
            vehicle = Vehicle.create_random(
                self.road,
                speed=25,
                lane_id=self.config["initial_lane_id"],
                spacing=self.config["ego_spacing"]
            )
            vehicle = self.action_type.vehicle_class(self.road, vehicle.position, vehicle.heading, vehicle.speed)
            self.controlled_vehicles.append(vehicle)
            self.road.vehicles.append(vehicle)

            for _ in range(others):
                vehicle = other_vehicles_type.create_random(self.road, spacing=1 / self.config["vehicles_density"])
                vehicle.randomize_behavior()
                self.road.vehicles.append(vehicle)

highway_env.HighwayEnv._create_vehicles = _create_vehicles

######## Configuration ########
lane_diff = 4 # Distance lanes are apart from each other
lanes_count = 4 # Number of lanes
use_absolute_lanes = True # Whether or not to label lanes as absolute or relative to current vehicle lane
KinematicObservation.normalize_obs = lambda self, df: df # Don't normalize values

steer_err = 0.01
acc_err = 1

env = gym.make('highway-fast-v0')
env.config['simulation_frequency']=24
env.config['policy_frequency']=8 # Runs once every 3 simulation steps
env.config['lanes_count']=lanes_count
env.config['initial_lane_id'] = 0
# env.configure({
    # "manual_control": True
# })

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

def plunder(ego, closest, ha):
    x, l_x, f_x, r_x = ego[1], closest[0][1], closest[1][1], closest[2][1] 
    vx = ego[3]
    f_vx = closest[1][3]
    l_vx = closest[0][3]
    r_vx = closest[2][3]

    if ha == env.action_type.actions_indexes["FASTER"] and sample(logistic2(f_vx, 1000000000.000000, 1.000000)):
        return env.action_type.actions_indexes["LANE_LEFT"]
    if ha == env.action_type.actions_indexes["FASTER"] and sample(logistic2(DividedBy(Minus(x, r_x), vx), -1.028642, -35.895672)):
        return env.action_type.actions_indexes["LANE_RIGHT"]
    if ha == env.action_type.actions_indexes["FASTER"] and sample(logistic2(Minus(r_x, f_x), -38.665745, 0.395329)):
        return env.action_type.actions_indexes["SLOWER"]
    if ha == env.action_type.actions_indexes["LANE_LEFT"] and sample(logistic2(f_x, 1000000000.000000, 1.000000)):
        return env.action_type.actions_indexes["FASTER"]
    if ha == env.action_type.actions_indexes["LANE_LEFT"] and sample(logistic2(f_x, 1000000000.000000, 1.000000)):
        return env.action_type.actions_indexes["LANE_RIGHT"]
    if ha == env.action_type.actions_indexes["LANE_LEFT"] and sample(logistic2(vx, 1000000000.000000, 1.000000)):
        return env.action_type.actions_indexes["SLOWER"]
    if ha == env.action_type.actions_indexes["LANE_RIGHT"] and sample(logistic2(Minus(r_x, x), 8.462489, -0.367737)):
        return env.action_type.actions_indexes["FASTER"]
    if ha == env.action_type.actions_indexes["LANE_RIGHT"] and sample(logistic2(r_vx, 1000000000.000000, 1.000000)):
        return env.action_type.actions_indexes["LANE_LEFT"]
    if ha == env.action_type.actions_indexes["LANE_RIGHT"] and sample(logistic2(DividedBy(Minus(x, r_x), l_vx), -0.377488, 10.546384)):
        return env.action_type.actions_indexes["SLOWER"]
    if ha == env.action_type.actions_indexes["SLOWER"] and sample(logistic2(Minus(x, f_x), -39.469845, -0.228033)):
        return env.action_type.actions_indexes["FASTER"]
    if ha == env.action_type.actions_indexes["SLOWER"] and sample(logistic2(vx, 1000000000.000000, 1.000000)):
        return env.action_type.actions_indexes["LANE_LEFT"]
    if ha == env.action_type.actions_indexes["SLOWER"] and sample(logistic2(Minus(x, r_x), -43.102879, -0.396746)):
        return env.action_type.actions_indexes["LANE_RIGHT"]

    return ha

def oneshot(ego, closest, ha):
    x, l_x, f_x, r_x = ego[1], closest[0][1], closest[1][1], closest[2][1] 
    vx = ego[3]
    f_vx = closest[1][3]
    l_vx = closest[0][3]
    r_vx = closest[2][3]

    if ha == env.action_type.actions_indexes["FASTER"] and sample(logistic2(l_x, -100.107910, -0.016411)):
        return env.action_type.actions_indexes["LANE_LEFT"]
    if ha == env.action_type.actions_indexes["FASTER"] and And(sample(logistic2(Minus(x, r_x), -32.109848, -0.146741)), sample(logistic2(f_vx, 30.627829, -0.135053))):
        return env.action_type.actions_indexes["LANE_RIGHT"]
    if ha == env.action_type.actions_indexes["FASTER"] and sample(logistic2(Minus(x, f_x), 1.049113, 0.055120)):
        return env.action_type.actions_indexes["SLOWER"]
    if ha == env.action_type.actions_indexes["LANE_LEFT"] and And(sample(logistic2(l_x, 320.450195, -0.055657)), sample(logistic2(DividedBy(l_x, vx), 8.430084, 65.953445))):
        return env.action_type.actions_indexes["FASTER"]
    if ha == env.action_type.actions_indexes["LANE_LEFT"] and And(sample(logistic2(r_vx, 25.450960, -7.530059)), sample(logistic2(Minus(x, f_x), -29.922413, -0.124320))):
        return env.action_type.actions_indexes["LANE_RIGHT"]
    if ha == env.action_type.actions_indexes["LANE_LEFT"] and And(sample(logistic2(Minus(x, r_x), 0.498732, -4.617277)), sample(logistic2(x, 183.704041, 4.327327))):
        return env.action_type.actions_indexes["SLOWER"]
    if ha == env.action_type.actions_indexes["LANE_RIGHT"] and sample(logistic2(Minus(x, r_x), 6.796582, 0.099759)):
        return env.action_type.actions_indexes["FASTER"]
    if ha == env.action_type.actions_indexes["LANE_RIGHT"] and sample(logistic2(x, 649.382080, 0.016938)):
        return env.action_type.actions_indexes["LANE_LEFT"]
    if ha == env.action_type.actions_indexes["LANE_RIGHT"] and sample(logistic2(Minus(r_x, x), -1.852137, -0.146202)):
        return env.action_type.actions_indexes["SLOWER"]
    if ha == env.action_type.actions_indexes["SLOWER"] and sample(logistic2(Minus(f_x, r_x), 79.645203, 0.072870)):
        return env.action_type.actions_indexes["FASTER"]
    if ha == env.action_type.actions_indexes["SLOWER"] and sample(logistic2(vx, 13.007671, -0.723762)):
        return env.action_type.actions_indexes["LANE_LEFT"]
    if ha == env.action_type.actions_indexes["SLOWER"] and sample(logistic2(Minus(r_x, x), 44.784977, 0.124707)):
        return env.action_type.actions_indexes["LANE_RIGHT"]
    return ha

def greedy(ego, closest, ha):
    x, l_x, f_x, r_x = ego[1], closest[0][1], closest[1][1], closest[2][1] 
    vx = ego[3]
    f_vx = closest[1][3]
    l_vx = closest[0][3]
    r_vx = closest[2][3]

    if ha == env.action_type.actions_indexes["FASTER"] and sample(logistic2(l_vx, 1.156561, -0.135662)):
        return env.action_type.actions_indexes["LANE_LEFT"]
    if ha == env.action_type.actions_indexes["FASTER"] and sample(logistic2(Minus(r_x, x), 40.128521, 0.063850)):
        return env.action_type.actions_indexes["LANE_RIGHT"]
    if ha == env.action_type.actions_indexes["FASTER"] and sample(logistic2(Minus(r_x, f_x), -39.489910, 0.042046)):
        return env.action_type.actions_indexes["SLOWER"]
    if ha == env.action_type.actions_indexes["LANE_LEFT"] and And(sample(logistic2(f_vx, 20.859222, 0.303888)), sample(logistic2(l_vx, 21.757748, -78.955055))):
        return env.action_type.actions_indexes["FASTER"]
    if ha == env.action_type.actions_indexes["LANE_LEFT"] and sample(logistic2(Minus(x, r_x), -12.149882, -0.113389)):
        return env.action_type.actions_indexes["LANE_RIGHT"]
    if ha == env.action_type.actions_indexes["LANE_LEFT"] and sample(logistic2(x, -60.812881, 0.081150)):
        return env.action_type.actions_indexes["SLOWER"]
    if ha == env.action_type.actions_indexes["LANE_RIGHT"] and sample(logistic2(r_vx, 35.249874, 0.155746)):
        return env.action_type.actions_indexes["FASTER"]
    if ha == env.action_type.actions_indexes["LANE_RIGHT"] and sample(logistic2(x, 538.604370, 0.007168)):
        return env.action_type.actions_indexes["LANE_LEFT"]
    if ha == env.action_type.actions_indexes["LANE_RIGHT"] and sample(logistic2(r_vx, 27.792574, 0.231732)):
        return env.action_type.actions_indexes["SLOWER"]
    if ha == env.action_type.actions_indexes["SLOWER"] and sample(logistic2(Minus(f_x, r_x), 55.778164, 0.025799)):
        return env.action_type.actions_indexes["FASTER"]
    if ha == env.action_type.actions_indexes["SLOWER"] and sample(logistic2(r_vx, 49.745766, 0.096640)):
        return env.action_type.actions_indexes["LANE_LEFT"]
    if ha == env.action_type.actions_indexes["SLOWER"] and sample(logistic2(Minus(x, r_x), -11.800286, -0.165087)):
        return env.action_type.actions_indexes["LANE_RIGHT"]
    return ha

def ldips(ego, closest, ha):
    x, l_x, f_x, r_x = ego[1], closest[0][1], closest[1][1], closest[2][1] 
    vx = ego[3]
    f_vx = closest[1][3]
    l_vx = closest[0][3]
    r_vx = closest[2][3]

    if ha == env.action_type.actions_indexes["FASTER"] and Gt(r_x * r_x, 261204.812500):
        return env.action_type.actions_indexes["LANE_LEFT"]
    if ha == env.action_type.actions_indexes["FASTER"] and Gt(Minus(r_x, x), 39.133987):
        return env.action_type.actions_indexes["LANE_RIGHT"]
    if ha == env.action_type.actions_indexes["FASTER"] and And(Gt(Minus(x, f_x), -33.616005), Gt(vx, 25.438000)):
        return env.action_type.actions_indexes["SLOWER"]
    if ha == env.action_type.actions_indexes["LANE_LEFT"] and Gt(Minus(f_x, r_x), 41.872009):
        return env.action_type.actions_indexes["FASTER"]
    if ha == env.action_type.actions_indexes["LANE_LEFT"] and Or(Lt(vx, 19.781000), Lt(Minus(x, r_x), -18.211000)):
        return env.action_type.actions_indexes["LANE_RIGHT"]
    if ha == env.action_type.actions_indexes["LANE_LEFT"] and Or(Gt(l_vx, 33.523998), Gt(Plus(x, l_x), 598.777039)):
        return env.action_type.actions_indexes["SLOWER"]
    if ha == env.action_type.actions_indexes["LANE_RIGHT"] and And(Gt(Minus(x, r_x), -18.211000), Gt(x, 403.574005)):
        return env.action_type.actions_indexes["FASTER"]
    if ha == env.action_type.actions_indexes["LANE_RIGHT"] and Gt(vx, 40.094002):
        return env.action_type.actions_indexes["LANE_LEFT"]
    if ha == env.action_type.actions_indexes["LANE_RIGHT"] and And(Gt(x, 194.106995), Gt(Minus(x, r_x), -6.394012)):
        return env.action_type.actions_indexes["SLOWER"]
    if ha == env.action_type.actions_indexes["SLOWER"] and And(Lt(Minus(x, f_x), -50.355988), Gt(x, 204.595001)):
        return env.action_type.actions_indexes["FASTER"]
    if ha == env.action_type.actions_indexes["SLOWER"] and Gt(DividedBy(r_x, vx), 20.051121):
        return env.action_type.actions_indexes["LANE_LEFT"]
    if ha == env.action_type.actions_indexes["SLOWER"] and Gt(Minus(r_x, x), 15.737000):
        return env.action_type.actions_indexes["LANE_RIGHT"]
    return ha

# modified from https://github.com/eleurent/highway-env/blob/31881fbe45fd05dbd3203bb35419ff5fb1b7bc09/highway_env/vehicle/controller.py
# in this version, no extra latent state is stored (target_lane, target_speed)
TURN_HEADING = 0.15 # Target heading when turning
TURN_TARGET = 30 # How much to adjust when targeting a lane (higher = smoother)
max_velocity = 45 # Maximum velocity

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
        target_acc = -0.5

        # Attain rightmost heading
        target_heading = TURN_HEADING
    elif action == "LANE_LEFT":
        target_acc = -0.5

        # Attain leftmost heading
        target_heading = -TURN_HEADING

    target_steer = target_heading - self.heading

    if target_steer > last_la["steering"]:
        target_steer = min(target_steer, last_la["steering"] + 0.08)
    else:
        target_steer = max(target_steer, last_la["steering"] - 0.08)

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
success = 0
def runSim(iter):
    global success
    env.reset()
    graphics.LAST_HA = "FASTER"
    ha = env.action_type.actions_indexes[graphics.LAST_HA]
    obs_out = open("data" + str(iter) + ".csv", "w")
    obs_out.write("x, y, vx, vy, heading, l_x, l_y, l_vx, l_vy, l_heading, f_x, f_y, f_vx, f_vy, f_heading, r_x, r_y, r_vx, r_vy, r_heading, LA.steer, LA.acc, HA\n")

    for _ in range(75):

        obs, reward, done, truncated, info = env.step(ha)
        env.render()

        # Pre-process observations
        lane_class = classifyLane(obs)
        closest = closestVehicles(obs, lane_class)

        # # Run ASP
        # ha = env.action_type.actions_indexes[graphics.LAST_HA]
        ha = greedy(obs[0], closest, ha)

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

    if(laneFinder(obs[0][2]) == 3):
        success += 1
        print(success)

    obs_out.close()

for iter in range(100):
    runSim(iter)

print(success)
