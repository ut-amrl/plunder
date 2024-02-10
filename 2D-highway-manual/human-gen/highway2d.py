import gymnasium as gym
import highway_env
from highway_env.envs import ControlledVehicle, Vehicle
from highway_env.envs.common.observation import KinematicObservation
import numpy as np
import random
from typing import Union
import pygame

env = gym.make('highway-v0', render_mode='rgb_array')

######## Configuration ########
lane_diff = 4 # Distance lanes are apart from each other
lanes_count = 4 # Number of lanes
use_absolute_lanes = True # Whether or not to label lanes as absolute or relative to current vehicle lane
KinematicObservation.normalize_obs = lambda self, df: df # Don't normalize values

steer_err = 0.01
acc_err = 1

env.config['simulation_frequency']=24
env.config['policy_frequency']=6 # Runs once every 4 simulation steps
env.config['lanes_count']=lanes_count
env.config['manual_control']=True

# Pygame config
pygame.init()
pygame.joystick.init()

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

max_velocity = 25 # Maximum velocity
min_velocity = 20 # Minimum velocity
# Inputs from controller
input_acceleration = 0 
input_steering = 0

def run_la(self, action: Union[dict, str] = None, step = True) -> None:
    global input_acceleration, input_steering, max_velocity, min_velocity
    
    if self.speed > max_velocity:
        input_acceleration = min(input_acceleration, 0)
    if self.speed < min_velocity:
        input_acceleration = max(input_acceleration, 0)

    la = {"acceleration" : input_acceleration, "steering": input_steering}

    if step:
        Vehicle.act(self, la)

    return la

ControlledVehicle.act = run_la

######## Simulation ########
def runSim(iter):
    global input_acceleration, input_steering
    env.reset()
    obs_out = open("data" + str(iter) + ".csv", "w")
    obs_out.write("x, y, vx, vy, heading, l_x, l_y, l_vx, l_vy, l_heading, f_x, f_y, f_vx, f_vy, f_heading, r_x, r_y, r_vx, r_vy, r_heading, LA.steer, LA.acc, HA\n")

    for _ in range(150):
        
        for _ in pygame.event.get(): # User did something.
            pass
        
        # Get count of joysticks.
        joystick_count = pygame.joystick.get_count()
        assert joystick_count <= 1, "Multiple joysticks detected"
        if joystick_count > 0:
            joystick = pygame.joystick.Joystick(0)
            joystick.init()

            k_acc = 4
            k_steer = 0.02

            input_acceleration = -joystick.get_axis(1) * k_acc
            input_steering = joystick.get_axis(3) * k_steer

            print(str(input_acceleration) + " & " + str(input_steering))

        obs, reward, done, truncated, info = env.step(0)
        env.render()

        # Pre-process observations
        lane_class = classifyLane(obs)
        closest = closestVehicles(obs, lane_class)

        # Run motor model
        la = run_la(env.vehicle, ACTIONS_ALL[0], False)

        # Ego vehicle
        for prop in obs[0][1:]:
            obs_out.write(str(round(prop, 3))+", ")
        
        # Nearby vehicles
        for v in closest:
            for prop in v[1:]:
                obs_out.write(str(round(prop, 3))+", ")
        obs_out.write(str(round(la['steering'], 3))+", ")
        obs_out.write(str(round(la['acceleration'], 3))+", ")
        obs_out.write(str(ACTION_REORDER[0]))
        obs_out.write("\n")

    obs_out.close()

for iter in range(20):
    runSim(iter)

pygame.quit()