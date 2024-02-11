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


env = gym.make('highway-fast-v0')

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

def Times(x, y):
    return x * y

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
        right_clear = sample(logistic(-5, -40, r_x - x)) or sample(logistic(0.5, 40, (r_x - x) / vx)) # time to collision
        if right_clear: # No car on the right: merge right
            return env.action_type.actions_indexes["LANE_RIGHT"]
        if front_clear: 
            return env.action_type.actions_indexes["FASTER"]

        # Nowhere to go: decelerate
        return env.action_type.actions_indexes["SLOWER"]

    right_clear = sample(logistic(-5, -40, r_x - x)) or sample(logistic(1, 40, (r_x - x) / vx)) # time to collision
    if right_clear: # No car on the right: merge right
        return env.action_type.actions_indexes["LANE_RIGHT"]
    if front_clear: 
        return env.action_type.actions_indexes["FASTER"]

    # Nowhere to go: decelerate
    return env.action_type.actions_indexes["SLOWER"]

def plunder(ego, closest, ha):
    x, l_x, f_x, r_x = ego[1], closest[0][1], closest[1][1], closest[2][1] 
    y, l_y, f_y, r_y = ego[2], closest[0][2], closest[1][2], closest[2][2] 
    vx = ego[3]
    f_vx = closest[1][3]
    l_vx = closest[0][3]
    r_vx = closest[2][3]

    if ha == env.action_type.actions_indexes['FASTER'] and sample(logistic2(x, 1043.003418, 0.010987)):
        return env.action_type.actions_indexes['LANE_LEFT']
    if ha == env.action_type.actions_indexes['FASTER'] and Or(sample(logistic2(Minus(r_x, x), 24.256481, 0.261414)), sample(logistic2(Minus(r_x, x), -7.083443, -10.012507))):
        return env.action_type.actions_indexes['LANE_RIGHT']
    if ha == env.action_type.actions_indexes['FASTER'] and And(sample(logistic2(Minus(r_x, f_x), -5.321127, 27.355986)), sample(logistic2(Minus(y, r_y), -2.810756, 45.921741))):
        return env.action_type.actions_indexes['SLOWER']
    if ha == env.action_type.actions_indexes['LANE_LEFT'] and And(sample(logistic2(Plus(y, r_y), 27.675999, -28.561993)), Or(sample(logistic2(Minus(r_y, y), 4.606244, -14.761892)), Or(sample(logistic2(Minus(f_x, r_x), 131.751694, 4.325001)), sample(logistic2(f_x, 364.858917, -1.114238))))):
        return env.action_type.actions_indexes['FASTER']
    if ha == env.action_type.actions_indexes['LANE_LEFT'] and sample(logistic2(f_y, 1000000000.000000, 1.000000)):
        return env.action_type.actions_indexes['LANE_RIGHT']
    if ha == env.action_type.actions_indexes['LANE_LEFT'] and sample(logistic2(f_x, 389.029755, -1.134043)):
        return env.action_type.actions_indexes['SLOWER']
    if ha == env.action_type.actions_indexes['LANE_RIGHT'] and Or(sample(logistic2(Minus(r_y, Plus(x, f_y)), -407.739685, -2.902149)), sample(logistic2(f_y, 13.234916, 0.499033))):
        return env.action_type.actions_indexes['FASTER']
    if ha == env.action_type.actions_indexes['LANE_RIGHT'] and sample(logistic2(Minus(f_x, r_x), 168.824478, 0.089852)):
        return env.action_type.actions_indexes['LANE_LEFT']
    if ha == env.action_type.actions_indexes['LANE_RIGHT'] and sample(logistic2(Times(x, y), 4744.125977, 0.026073)):
        return env.action_type.actions_indexes['SLOWER']
    return ha

def oneshot(ego, closest, ha):
    x, l_x, f_x, r_x = ego[1], closest[0][1], closest[1][1], closest[2][1] 
    y, l_y, f_y, r_y = ego[2], closest[0][2], closest[1][2], closest[2][2] 
    vx = ego[3]
    f_vx = closest[1][3]
    l_vx = closest[0][3]
    r_vx = closest[2][3]

    if ha == env.action_type.actions_indexes['FASTER'] and sample(logistic2(r_x, 782.265015, 0.012602)):
        return env.action_type.actions_indexes['LANE_LEFT']
    if ha == env.action_type.actions_indexes['FASTER'] and And(sample(logistic2(r_y, 3.998074, -0.539815)), sample(logistic2(y, 0.040286, 74.949547))):
        return env.action_type.actions_indexes['LANE_RIGHT']
    if ha == env.action_type.actions_indexes['FASTER'] and And(sample(logistic2(Minus(y, f_y), 2.908221, 2.255324)), sample(logistic2(y, 12.994962, -51.274734))):
        return env.action_type.actions_indexes['SLOWER']
    if ha == env.action_type.actions_indexes['LANE_LEFT'] and And(sample(logistic2(f_x, 498.601288, 0.008343)), sample(logistic2(Minus(y, f_y), 0.756004, -1.416386))):
        return env.action_type.actions_indexes['FASTER']
    if ha == env.action_type.actions_indexes['LANE_LEFT'] and And(sample(logistic2(Minus(x, f_x), -125.834984, 0.112066)), sample(logistic2(y, -3.719848, -0.246040))):
        return env.action_type.actions_indexes['LANE_RIGHT']
    if ha == env.action_type.actions_indexes['LANE_LEFT'] and And(sample(logistic2(Minus(y, f_y), 1.670084, 1.428553)), sample(logistic2(x, 321.862061, 0.935113))):
        return env.action_type.actions_indexes['SLOWER']
    if ha == env.action_type.actions_indexes['LANE_RIGHT'] and And(sample(logistic2(Minus(x, r_x), 0.318488, -1.804582)), sample(logistic2(Minus(x, r_x), 8.807152, 0.077301))):
        return env.action_type.actions_indexes['FASTER']
    if ha == env.action_type.actions_indexes['LANE_RIGHT'] and sample(logistic2(f_y, 26.429953, 0.248994)):
        return env.action_type.actions_indexes['LANE_LEFT']
    if ha == env.action_type.actions_indexes['LANE_RIGHT'] and sample(logistic2(y, 16.086760, 0.689301)):
        return env.action_type.actions_indexes['SLOWER']
    if ha == env.action_type.actions_indexes['SLOWER'] and sample(logistic2(Minus(y, f_y), -0.701414, -4.562846)):
        return env.action_type.actions_indexes['FASTER']
    if ha == env.action_type.actions_indexes['SLOWER'] and sample(logistic2(f_x, 446.782928, 0.173304)):
        return env.action_type.actions_indexes['LANE_LEFT']
    if ha == env.action_type.actions_indexes['SLOWER'] and And(sample(logistic2(r_y, -10.542195, -0.019090)), sample(logistic2(Minus(x, f_x), -23.800596, -0.353469))):
        return env.action_type.actions_indexes['LANE_RIGHT']
    return ha

def greedy(ego, closest, ha):
    x, l_x, f_x, r_x = ego[1], closest[0][1], closest[1][1], closest[2][1] 
    y, l_y, f_y, r_y = ego[2], closest[0][2], closest[1][2], closest[2][2] 
    vx = ego[3]
    f_vx = closest[1][3]
    l_vx = closest[0][3]
    r_vx = closest[2][3]

    if ha == env.action_type.actions_indexes['FASTER'] and And(sample(logistic2(Minus(y, f_y), 3.191783, 1.140633)), sample(logistic2(y, 3.786633, 46.292389))):
        return env.action_type.actions_indexes['LANE_LEFT']
    if ha == env.action_type.actions_indexes['FASTER'] and And(sample(logistic2(Minus(x, r_x), -38.194477, -0.070629)), sample(logistic2(Plus(x, y), 412.802490, -12.517858))):
        return env.action_type.actions_indexes['LANE_RIGHT']
    if ha == env.action_type.actions_indexes['FASTER'] and sample(logistic2(x, 1000000000.000000, 1.000000)):
        return env.action_type.actions_indexes['SLOWER']
    if ha == env.action_type.actions_indexes['LANE_LEFT'] and And(sample(logistic2(Minus(x, f_x), -28.455622, -0.036815)), sample(logistic2(Minus(x, r_x), -13.841810, 26.210276))):
        return env.action_type.actions_indexes['FASTER']
    if ha == env.action_type.actions_indexes['LANE_LEFT'] and And(sample(logistic2(f_y, 10.546297, -0.221200)), sample(logistic2(Minus(y, f_x), -300.946655, -9.962460))):
        return env.action_type.actions_indexes['LANE_RIGHT']
    if ha == env.action_type.actions_indexes['LANE_LEFT'] and sample(logistic2(x, 1000000000.000000, 1.000000)):
        return env.action_type.actions_indexes['SLOWER']
    if ha == env.action_type.actions_indexes['LANE_RIGHT'] and And(sample(logistic2(Minus(x, r_x), 7.312887, -15.160950)), sample(logistic2(Minus(x, r_x), 1.878983, 0.099254))):
        return env.action_type.actions_indexes['FASTER']
    if ha == env.action_type.actions_indexes['LANE_RIGHT'] and And(sample(logistic2(Plus(x, f_x), 543.351196, 78.232239)), sample(logistic2(Minus(y, r_x), -236.922424, 0.042448))):
        return env.action_type.actions_indexes['LANE_LEFT']
    if ha == env.action_type.actions_indexes['LANE_RIGHT'] and And(sample(logistic2(Minus(y, f_y), 2.206622, 1.147135)), sample(logistic2(r_x, 325.828247, 0.124905))):
        return env.action_type.actions_indexes['SLOWER']
    if ha == env.action_type.actions_indexes['SLOWER'] and sample(logistic2(x, 1000000000.000000, 1.000000)):
        return env.action_type.actions_indexes['FASTER']
    if ha == env.action_type.actions_indexes['SLOWER'] and sample(logistic2(f_x, 434.061646, 1.885659)):
        return env.action_type.actions_indexes['LANE_LEFT']
    if ha == env.action_type.actions_indexes['SLOWER'] and And(sample(logistic2(y, 8.160023, -0.633028)), sample(logistic2(Minus(y, r_y), -2.458457, -168.340668))):
        return env.action_type.actions_indexes['LANE_RIGHT']
    return ha

def ldips(ego, closest, ha):
    x, l_x, f_x, r_x = ego[1], closest[0][1], closest[1][1], closest[2][1] 
    y, l_y, f_y, r_y = ego[2], closest[0][2], closest[1][2], closest[2][2] 
    vx = ego[3]
    f_vx = closest[1][3]
    l_vx = closest[0][3]
    r_vx = closest[2][3]

    if ha == env.action_type.actions_indexes['FASTER'] and Gt(Minus(r_y, r_x), -226.710999):
        return env.action_type.actions_indexes['LANE_LEFT']
    if ha == env.action_type.actions_indexes['FASTER'] and Or(Lt(Plus(x, x), 442.381989), Gt(Minus(x, r_x), 7.053986)):
        return env.action_type.actions_indexes['LANE_RIGHT']
    if ha == env.action_type.actions_indexes['FASTER'] and And(Lt(Minus(x, f_x), 14.541992), Gt(Minus(x, f_x), -9.506012)):
        return env.action_type.actions_indexes['SLOWER']
    if ha == env.action_type.actions_indexes['LANE_LEFT'] and Or(Gt(y, 13.709000), Gt(Plus(x, y), 429.056000)):
        return env.action_type.actions_indexes['FASTER']
    if ha == env.action_type.actions_indexes['LANE_LEFT'] and And(Lt(Plus(y, r_x), 337.194000), Gt(x, 321.601990)):
        return env.action_type.actions_indexes['LANE_RIGHT']
    if ha == env.action_type.actions_indexes['LANE_LEFT'] and And(Gt(x, 331.510010), Gt(Minus(y, r_x), -342.792023)):
        return env.action_type.actions_indexes['SLOWER']
    if ha == env.action_type.actions_indexes['LANE_RIGHT'] and And(Lt(r_y, 16.850000), Gt(y, 10.452999)):
        return env.action_type.actions_indexes['FASTER']
    if ha == env.action_type.actions_indexes['LANE_RIGHT'] and And(Gt(f_x, 296.690002), Gt(Minus(y, f_x), -293.684998)):
        return env.action_type.actions_indexes['LANE_LEFT']
    if ha == env.action_type.actions_indexes['LANE_RIGHT'] and And(Gt(f_x, 371.557007), Gt(r_y, 16.753992)):
        return env.action_type.actions_indexes['SLOWER']
    if ha == env.action_type.actions_indexes['SLOWER'] and And(Lt(Plus(x, f_x), 662.135010), Gt(y, 3.020000)):
        return env.action_type.actions_indexes['FASTER']
    if ha == env.action_type.actions_indexes['SLOWER'] and Gt(Times(r_x, r_y), 7012.649902):
        return env.action_type.actions_indexes['LANE_LEFT']
    if ha == env.action_type.actions_indexes['SLOWER'] and Gt(Minus(x, r_x), 8.604004):
        return env.action_type.actions_indexes['LANE_RIGHT']
    return ha

# modified from https://github.com/eleurent/highway-env/blob/31881fbe45fd05dbd3203bb35419ff5fb1b7bc09/highway_env/vehicle/controller.py
# in this version, no extra latent state is stored (target_lane, target_speed)
TURN_HEADING = 0.1 # Target heading when turning
TURN_TARGET = 30 # How much to adjust when targeting a lane (higher = smoother)
max_velocity = 25 # Maximum velocity
min_velocity = 15 # Turning velocity

last_action = "FASTER"
def run_la(self, action: Union[dict, str] = None, step = True, closest = None) -> None:
    global last_action

    target_acc = 0.0
    target_steer = 0.0

    if action == None:
        action = last_action

    last_action = action

    if action == "FASTER":
        # Attain max speed
        target_acc = 4

        # Follow current lane
        target_y = laneFinder(self.position[1]) * lane_diff
        target_heading = np.arctan((target_y - self.position[1]) / TURN_TARGET)
        target_steer = max(min(target_heading - self.heading, 0.02), -0.02)
    elif action == "SLOWER":
        target_acc = -4

        # Follow current lane
        target_y = laneFinder(self.position[1]) * lane_diff
        target_heading = np.arctan((target_y - self.position[1]) / TURN_TARGET)
        target_steer = max(min(target_heading - self.heading, 0.02), -0.02)
    elif action == "LANE_RIGHT":
        target_acc = 0
        target_steer = max(min(TURN_HEADING - self.heading, 0.03), 0.0)
    elif action == "LANE_LEFT":
        target_acc = 0
        target_steer = max(min(-TURN_HEADING - self.heading, 0.0), -0.03)

    if self.velocity[0] >= max_velocity - 0.01:
        target_acc = min(target_acc, 0.0)
    if self.velocity[0] <= min_velocity + 0.01:
        target_acc = max(target_acc, 0.0)

    la = {"steering": target_steer, "acceleration": target_acc }
    if step:
            Vehicle.act(self, la)

    return la

ControlledVehicle.act = run_la

######## Simulation ########
success = 0
def runSim(iter):
    global success
    env.reset()
    ha = env.action_type.actions_indexes["FASTER"]

    for _ in range(150):

        obs, reward, done, truncated, info = env.step(ha)
        env.render()

        # Pre-process observations
        lane_class = classifyLane(obs)
        closest = closestVehicles(obs, lane_class)

        # Run ASP
        ha = ldips(obs[0], closest, ha)
        # Run motor model
        la = run_la(env.vehicle, ACTIONS_ALL[ha], False, closest)

    if(laneFinder(obs[0][2]) == 3):
        success += 1
        print(success)

for iter in range(200):
    runSim(iter)

print(success)
