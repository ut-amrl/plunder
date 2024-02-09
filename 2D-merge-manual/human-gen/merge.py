import gymnasium as gym
import highway_env
from highway_env.envs import ControlledVehicle, Vehicle, highway_env
from highway_env.envs.common.observation import KinematicObservation
from highway_env.envs.common import graphics
from highway_env.utils import near_split, class_from_path
from highway_env.road.road import Road
import numpy as np
import random
from typing import List, Tuple, Union, Optional
import pygame

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

# Allow vehicle to see vehicles behind it as well
def close_vehicles_to(
    self,
    vehicle: "kinematics.Vehicle",
    distance: float,
    count: Optional[int] = None,
    see_behind: bool = True,
    sort: bool = True,
    vehicles_only: bool = False,
) -> object:
    vehicles = [
        v
        for v in self.vehicles
        if np.linalg.norm(v.position - vehicle.position) < distance
        and v is not vehicle
        and (see_behind or -5 * vehicle.LENGTH < vehicle.lane_distance_to(v))
    ]

    objects_ = vehicles

    if sort:
        objects_ = sorted(objects_, key=lambda o: abs(vehicle.lane_distance_to(o)))
    if count:
        objects_ = objects_[:count]
    return objects_

highway_env.Road.close_vehicles_to = close_vehicles_to




env = gym.make('highway-v0', render_mode='rgb_array')

######## Configuration ########
lane_diff = 4 # Distance lanes are apart from each other
lanes_count = 4 # Number of lanes
use_absolute_lanes = True # Whether or not to label lanes as absolute or relative to current vehicle lane
KinematicObservation.normalize_obs = lambda self, df: df # Don't normalize values

steer_err = 0.01
acc_err = 1

env.config['simulation_frequency']=24
env.config['policy_frequency']=8 # Runs once every 3 simulation steps
env.config['lanes_count']=lanes_count
env.config['initial_lane_id'] = 0
env.config['manual_control'] = True

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
min_velocity = 15 # Minimum velocity

# Inputs from controller
input_acceleration = 0 
input_steering = 0

last_action = "FASTER"
last_target = 0
last_la = {"steering": 0, "acceleration": 0 }

def run_la(self, action: Union[dict, str] = None, step = True) -> None:
    global input_acceleration, input_steering, max_velocity, min_velocity
    
    la = {"acceleration" : input_acceleration, "steering": input_steering}

    if step:
        sp = input_acceleration
        if self.speed > max_velocity:
            sp = min(input_acceleration, 0)
        if self.speed < min_velocity:
            sp = max(input_acceleration, 0)
        act = {"acceleration" : sp, "steering": input_steering}
        Vehicle.act(self, act)

    return la

ControlledVehicle.act = run_la

######## Simulation ########
def runSim(iter):
    global input_acceleration, input_steering
    env.reset()
    obs_out = open("data" + str(iter) + ".csv", "w")
    obs_out.write("x, y, vx, vy, heading, l_x, l_y, l_vx, l_vy, l_heading, f_x, f_y, f_vx, f_vy, f_heading, r_x, r_y, r_vx, r_vy, r_heading, LA.steer, LA.acc, HA\n")

    for _ in range(80):

        for _ in pygame.event.get(): # User did something.
            pass

        # Get count of joysticks.
        joystick_count = pygame.joystick.get_count()
        assert joystick_count <= 1, "Multiple joysticks detected"
        if joystick_count > 0:
            joystick = pygame.joystick.Joystick(0)
            joystick.init()

            k_acc = 4
            k_steer = 0.03

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
        la['steering'] = np.random.normal(la['steering'], steer_err)
        la['acceleration'] = np.random.normal(la['acceleration'], acc_err)

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
