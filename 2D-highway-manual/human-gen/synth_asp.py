import gymnasium as gym
import highway_env
from highway_env.envs import ControlledVehicle, Vehicle
from highway_env.envs.common.observation import KinematicObservation
import numpy as np
import random
from typing import Union

env = gym.make('highway-fast-v0', render_mode='rgb_array')

######## Configuration ########
lane_diff = 4 # Distance lanes are apart from each other
lanes_count = 8 # Number of lanes
use_absolute_lanes = True # Whether or not to label lanes as absolute or relative to current vehicle lane
KinematicObservation.normalize_obs = lambda self, df: df # Don't normalize values

env.config['simulation_frequency']=24
env.config['policy_frequency']=6 # Runs once every 4 simulation steps
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
    l_vx = closest[0][3]
    f_vx = closest[1][3]
    r_vx = closest[2][3]

    if ha == env.action_type.actions_indexes['FASTER'] and And(sample(logistic2(f_vx, 35.021538, -21.117664)), And(sample(logistic2(Minus(f_x, l_x), -28.455282, -0.111654)), And(sample(logistic2(f_vx, 21.367447, -32.117924)), sample(logistic2(Minus(f_x, x), 22.931559, -0.621136))))):
        return env.action_type.actions_indexes['LANE_LEFT']
    if ha == env.action_type.actions_indexes['FASTER'] and sample(logistic2(DividedBy(Minus(f_x, x), f_vx), 0.903634, -9.377876)):
        return env.action_type.actions_indexes['LANE_RIGHT']
    if ha == env.action_type.actions_indexes['FASTER'] and Or(sample(logistic2(Minus(r_x, l_x), 142.693558, 26.650047)), Or(sample(logistic2(Plus(f_vx, r_vx), -108.861458, -0.039834)), And(sample(logistic2(vx, 24.874939, -61.811741)), sample(logistic2(Minus(f_x, Plus(l_x, r_x)), -557.253906, -0.012443))))):
        return env.action_type.actions_indexes['SLOWER']
    if ha == env.action_type.actions_indexes['LANE_LEFT'] and And(sample(logistic2(r_vx, 19.766947, 2.068509)), sample(logistic2(Minus(x, f_x), -31.710594, -0.294839))):
        return env.action_type.actions_indexes['FASTER']
    if ha == env.action_type.actions_indexes['LANE_LEFT'] and Or(sample(logistic2(DividedBy(x, l_vx), 34.269012, 245.851440)), And(sample(logistic2(x, 235.960114, -0.653907)), Or(sample(logistic2(l_vx, 126.732285, -0.239940)), sample(logistic2(Plus(l_vx, r_vx), 44.333492, 2.931920))))):
        return env.action_type.actions_indexes['LANE_RIGHT']
    if ha == env.action_type.actions_indexes['LANE_LEFT'] and sample(logistic2(vx, 24.829386, -86.008926)):
        return env.action_type.actions_indexes['SLOWER']
    if ha == env.action_type.actions_indexes['LANE_RIGHT'] and And(sample(logistic2(l_vx, 20.476099, 0.709218)), sample(logistic2(Minus(x, f_x), -26.473980, -0.609106))):
        return env.action_type.actions_indexes['FASTER']
    if ha == env.action_type.actions_indexes['LANE_RIGHT'] and And(sample(logistic2(vx, 25.023340, 8.410436)), And(sample(logistic2(Plus(l_vx, r_vx), 41.128242, -2.412731)), sample(logistic2(f_vx, 20.260880, -5.940400)))):
        return env.action_type.actions_indexes['LANE_LEFT']
    if ha == env.action_type.actions_indexes['LANE_RIGHT'] and sample(logistic2(Times(vx, Plus(Plus(vx, Plus(vx, l_vx)), f_vx)), 2233.293213, -0.184224)):
        return env.action_type.actions_indexes['SLOWER']
    if ha == env.action_type.actions_indexes['SLOWER'] and sample(logistic2(Times(vx, Plus(Plus(vx, vx), f_vx)), 1764.555542, 0.051010)):
        return env.action_type.actions_indexes['FASTER']
    if ha == env.action_type.actions_indexes['SLOWER'] and And(sample(logistic2(DividedBy(Minus(r_x, f_x), l_vx), 1.813058, 0.469895)), sample(logistic2(vx, 24.549904, 27.209173))):
        return env.action_type.actions_indexes['LANE_LEFT']
    if ha == env.action_type.actions_indexes['SLOWER'] and Or(sample(logistic2(Plus(r_vx, r_vx), 37.099731, -12.132344)), sample(logistic2(vx, 25.158022, 46.733471))):
        return env.action_type.actions_indexes['LANE_RIGHT']
    return ha

def oneshot(ego, closest, ha):
    x, l_x, f_x, r_x = ego[1], closest[0][1], closest[1][1], closest[2][1] 
    vx = ego[3]
    f_vx = closest[1][3]
    l_vx = closest[0][3]
    r_vx = closest[2][3]

    if ha == env.action_type.actions_indexes['FASTER'] and sample(logistic2(Minus(f_x, x), -14.610590, -0.085343)):
        return env.action_type.actions_indexes['LANE_LEFT']
    if ha == env.action_type.actions_indexes['FASTER'] and sample(logistic2(Minus(f_x, x), -46.063908, -0.050458)):
        return env.action_type.actions_indexes['LANE_RIGHT']
    if ha == env.action_type.actions_indexes['FASTER'] and sample(logistic2(Minus(f_x, x), -62.449100, -0.049097)):
        return env.action_type.actions_indexes['SLOWER']
    if ha == env.action_type.actions_indexes['LANE_LEFT'] and sample(logistic2(Minus(f_x, x), 125.835709, 0.026707)):
        return env.action_type.actions_indexes['FASTER']
    if ha == env.action_type.actions_indexes['LANE_LEFT'] and sample(logistic2(f_vx, 26.063660, 0.627788)):
        return env.action_type.actions_indexes['LANE_RIGHT']
    if ha == env.action_type.actions_indexes['LANE_LEFT'] and And(sample(logistic2(l_vx, 29.121233, 0.209708)), sample(logistic2(vx, 24.597170, -4.726738))):
        return env.action_type.actions_indexes['SLOWER']
    if ha == env.action_type.actions_indexes['LANE_RIGHT'] and sample(logistic2(Minus(x, f_x), -123.878181, -0.027582)):
        return env.action_type.actions_indexes['FASTER']
    if ha == env.action_type.actions_indexes['LANE_RIGHT'] and sample(logistic2(f_vx, 15.562055, -0.731648)):
        return env.action_type.actions_indexes['LANE_LEFT']
    if ha == env.action_type.actions_indexes['LANE_RIGHT'] and And(sample(logistic2(Minus(x, r_x), -1.519037, 0.042705)), sample(logistic2(vx, 22.264265, -1.129654))):
        return env.action_type.actions_indexes['SLOWER']
    if ha == env.action_type.actions_indexes['SLOWER'] and And(sample(logistic2(vx, 28.344425, 0.347318)), sample(logistic2(Minus(x, f_x), -31.204630, -0.127223))):
        return env.action_type.actions_indexes['FASTER']
    if ha == env.action_type.actions_indexes['SLOWER'] and sample(logistic2(vx, 29.539448, 0.702288)):
        return env.action_type.actions_indexes['LANE_LEFT']
    if ha == env.action_type.actions_indexes['SLOWER'] and And(sample(logistic2(Minus(x, r_x), -54.775555, -0.046541)), sample(logistic2(vx, 26.805931, 0.576199))):
        return env.action_type.actions_indexes['LANE_RIGHT']
    return ha

def greedy(ego, closest, ha):
    x, l_x, f_x, r_x = ego[1], closest[0][1], closest[1][1], closest[2][1] 
    vx = ego[3]
    f_vx = closest[1][3]
    l_vx = closest[0][3]
    r_vx = closest[2][3]

    if ha == env.action_type.actions_indexes['FASTER'] and And(sample(logistic2(vx, 25.388979, 7.775516)), sample(logistic2(f_vx, 21.020950, -2.873578))):
        return env.action_type.actions_indexes['LANE_LEFT']
    if ha == env.action_type.actions_indexes['FASTER'] and sample(logistic2(l_vx, 8.448796, -0.189611)):
        return env.action_type.actions_indexes['LANE_RIGHT']
    if ha == env.action_type.actions_indexes['FASTER'] and sample(logistic2(vx, 21.694502, -0.950563)):
        return env.action_type.actions_indexes['SLOWER']
    if ha == env.action_type.actions_indexes['LANE_LEFT'] and sample(logistic2(Minus(f_x, l_x), 3.074440, 0.031570)):
        return env.action_type.actions_indexes['FASTER']
    if ha == env.action_type.actions_indexes['LANE_LEFT'] and sample(logistic2(x, 1000000000.000000, 1.000000)):
        return env.action_type.actions_indexes['LANE_RIGHT']
    if ha == env.action_type.actions_indexes['LANE_LEFT'] and And(sample(logistic2(vx, 24.822018, -11.137282)), sample(logistic2(l_vx, 20.995428, -38.177990))):
        return env.action_type.actions_indexes['SLOWER']
    if ha == env.action_type.actions_indexes['LANE_RIGHT'] and And(sample(logistic2(r_vx, 23.360723, -0.315689)), sample(logistic2(Minus(x, f_x), -25.135120, -0.348444))):
        return env.action_type.actions_indexes['FASTER']
    if ha == env.action_type.actions_indexes['LANE_RIGHT'] and sample(logistic2(r_vx, 19.237362, -197.554749)):
        return env.action_type.actions_indexes['LANE_LEFT']
    if ha == env.action_type.actions_indexes['LANE_RIGHT'] and And(sample(logistic2(Minus(x, r_x), 10.074911, 0.058805)), sample(logistic2(Times(vx, f_vx), 517.332825, 21.826927))):
        return env.action_type.actions_indexes['SLOWER']
    if ha == env.action_type.actions_indexes['SLOWER'] and And(sample(logistic2(Times(vx, f_vx), 492.959900, 0.016580)), sample(logistic2(f_vx, 19.323608, 1127.358398))):
        return env.action_type.actions_indexes['FASTER']
    if ha == env.action_type.actions_indexes['SLOWER'] and And(sample(logistic2(Minus(x, r_x), -9.506098, -14.581052)), sample(logistic2(Minus(x, f_x), -44.926651, -0.080250))):
        return env.action_type.actions_indexes['LANE_LEFT']
    if ha == env.action_type.actions_indexes['SLOWER'] and And(sample(logistic2(x, 580.906860, 0.653624)), sample(logistic2(vx, 24.942379, 194.833572))):
        return env.action_type.actions_indexes['LANE_RIGHT']
    return ha

def ldips(ego, closest, ha):
    x, l_x, f_x, r_x = ego[1], closest[0][1], closest[1][1], closest[2][1] 
    vx = ego[3]
    f_vx = closest[1][3]
    l_vx = closest[0][3]
    r_vx = closest[2][3]

    if ha == env.action_type.actions_indexes['FASTER'] and And(Lt(f_x, 439.911987), Gt(x, 411.147003)):
        return env.action_type.actions_indexes['LANE_LEFT']
    if ha == env.action_type.actions_indexes['FASTER'] and And(Gt(Minus(x, f_x), -17.955988), Gt(r_vx, 21.267000)):
        return env.action_type.actions_indexes['LANE_RIGHT']
    if ha == env.action_type.actions_indexes['FASTER'] and And(Lt(x, 689.025024), Gt(DividedBy(x, vx), 27.432137)):
        return env.action_type.actions_indexes['SLOWER']
    if ha == env.action_type.actions_indexes['LANE_LEFT'] and And(Lt(Plus(l_x, r_x), 569.890991), Gt(vx, 25.159000)):
        return env.action_type.actions_indexes['FASTER']
    if ha == env.action_type.actions_indexes['LANE_LEFT'] and And(Gt(r_x, 799.440979), Gt(r_vx, 19.886000)):
        return env.action_type.actions_indexes['LANE_RIGHT']
    if ha == env.action_type.actions_indexes['LANE_LEFT'] and And(Gt(DividedBy(x, vx), 37.989227), Gt(Plus(x, r_x), 1558.518921)):
        return env.action_type.actions_indexes['SLOWER']
    if ha == env.action_type.actions_indexes['LANE_RIGHT'] and And(Lt(f_vx, 20.719999), Gt(f_vx, 20.688999)):
        return env.action_type.actions_indexes['FASTER']
    if ha == env.action_type.actions_indexes['LANE_RIGHT'] and And(Lt(Plus(x, r_x), 1170.353149), Gt(DividedBy(x, vx), 22.732645)):
        return env.action_type.actions_indexes['LANE_LEFT']
    if ha == env.action_type.actions_indexes['LANE_RIGHT'] and And(Lt(vx, 24.879000), Gt(vx, 22.997999)):
        return env.action_type.actions_indexes['SLOWER']
    if ha == env.action_type.actions_indexes['SLOWER'] and And(Lt(r_x, 381.783997), Lt(l_vx, 20.033001)):
        return env.action_type.actions_indexes['FASTER']
    if ha == env.action_type.actions_indexes['SLOWER'] and And(Lt(Plus(x, f_x), 1190.265991), Gt(DividedBy(x, vx), 22.413401)):
        return env.action_type.actions_indexes['LANE_LEFT']
    if ha == env.action_type.actions_indexes['SLOWER'] and And(Lt(f_vx, 20.377001), Gt(f_vx, 20.354000)):
        return env.action_type.actions_indexes['LANE_RIGHT']
    return ha

# modified from https://github.com/eleurent/highway-env/blob/31881fbe45fd05dbd3203bb35419ff5fb1b7bc09/highway_env/vehicle/controller.py
# in this version, no extra latent state is stored (target_lane, target_speed)
TURN_HEADING = 0.1 # Target heading when turning
TURN_TARGET = 30 # How much to adjust when targeting a lane (higher = smoother)
max_velocity = 25 # Maximum velocity
min_velocity = 20 # Turning velocity

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
        target_steer = max(min(target_heading - self.heading, 0.015), -0.015)
    elif action == "SLOWER":
        target_acc = -4

        # Follow current lane
        target_y = laneFinder(self.position[1]) * lane_diff
        target_heading = np.arctan((target_y - self.position[1]) / TURN_TARGET)
        target_steer = max(min(target_heading - self.heading, 0.015), -0.015)
    elif action == "LANE_RIGHT":
        target_acc = 4
        target_steer = 0.02
    elif action == "LANE_LEFT":
        target_acc = 4
        target_steer = -0.02

    if self.velocity[0] >= max_velocity - 0.01:
        target_acc = min(target_acc, 0.0)
    if self.velocity[0] <= min_velocity + 0.01:
        target_acc = max(target_acc, 0.0)

    if target_steer > 0:
        target_steer = min(target_steer, TURN_HEADING - self.heading)
    if target_steer < 0:
        target_steer = max(target_steer, -TURN_HEADING - self.heading)

    la = {"steering": target_steer, "acceleration": target_acc }
    if step:
            Vehicle.act(self, la)

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
        ha = ldips(obs[0], closest, ha)
        # Run motor model
        la = run_la(env.vehicle, ACTIONS_ALL[ha], False, closest)

        if start < 0:
            start = obs[0][1]

    if(obs[0][3] > 10 and laneFinder(obs[0][2]) >= 0 and laneFinder(obs[0][2]) <= 7): # Positive velocity and in an existing lane
        success += 1
        print(success)
    dist += obs[0][1] - start

for iter in range(200):
    runSim(iter)

print(success)
print(dist / 200)