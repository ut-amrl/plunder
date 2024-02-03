import gymnasium as gym
import panda_gym
import time
import numpy as np
import pandas as pd
import random
import tensorflow
from tensorflow import keras
from keras.models import Sequential
from keras.layers import Dense, LSTM
from keras.callbacks import EarlyStopping
import math
from matplotlib import pyplot
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from matplotlib.figure import Figure
from pandas import DataFrame
from sklearn.preprocessing import MinMaxScaler, LabelEncoder
from scipy.stats import norm
from highway_env.envs import MDPVehicle, ControlledVehicle, Vehicle, highway_env
from highway_env.envs.common.observation import KinematicObservation
from highway_env.envs.common import graphics
from highway_env.utils import near_split, class_from_path
from typing import List, Tuple, Union, Optional


training_set = 10
validation_set = 30 # including training_set
train_time = 15000
patience = 800
sim_time = 150
samples = 50
folder = "../../baselines/supervised_learning_la/highway/"
vars_used = [
    "HA",
    "x",
    "y",
    "l_x",
    "f_x",
    "r_x",
    "v_x",
    "l_vx",
    "f_vx",
    "r_vx",
    "LA.steer",
    "LA.acc"
]
pred_var = ["LA.steer", "LA.acc"]
pv_range = [
    [-0.3, 0.3],
    [-30, 30]
]
pv_stddev = [0.01, 2]

numHA = 4

KP_H = 0.5 # Turning rate
TURN_HEADING = 0.15 # Target heading when turning
TURN_TARGET = 30 # How much to adjust when targeting a lane (higher = smoother)
max_velocity = 40 # Maximum velocity
turn_velocity = 30 # Turning velocity

def laneFinder(y):
    return round(y / 4)

def motor_model(ha, data, data_prev):
    target_acc = 0.0
    target_heading = 0.0
    if ha == 0:
        target_acc = max_velocity - data["vx"]

        target_y = laneFinder(data["y"]) * 4
        target_heading = np.arctan((target_y - data["y"]) / TURN_TARGET)
    elif ha == 1:
        target_acc = data["f_vx"] - data["vx"]

        target_y = laneFinder(data["y"]) * 4
        target_heading = np.arctan((target_y - data["y"]) / TURN_TARGET)
    elif ha == 2:
        target_acc = turn_velocity - data["vx"]
        target_heading = -TURN_HEADING
    else:
        target_acc = turn_velocity - data["vx"]
        target_heading = TURN_HEADING

    target_steer = target_heading - data["heading"]
    if(target_steer > data_prev["LA.steer"]):
        target_steer = min(target_steer, data_prev["LA.steer"] + 0.04)
    else:
        target_steer = max(target_steer, data_prev["LA.steer"] - 0.04)

    if(target_acc > data_prev["LA.acc"]):  
        target_acc = min(target_acc, data_prev["LA.acc"] + 4)
    else:
        target_acc = max(target_acc, data_prev["LA.acc"] - 6)

    return [target_steer, target_acc]



# Load model
model = keras.models.load_model(folder + "model_PT")

column_names = []

# Convert series to supervised learning
def series_to_supervised(data, n_in=4, n_out=1, dropnan=True):
    df = DataFrame(data)
    cols, names = list(), list()

    # Input sequence
    for i in range(n_in, 0, -1):
        cols.append(df.shift(i))
        names += [(label + '(t-%d)' % i) for label in data.columns]

    # Forecast sequence
    for i in range(0, n_out):
        cols.append(df.shift(-i))
        if i == 0:
            names += [(label + '(t)') for label in data.columns]
        else:
            names += [(label + '(t+%d)' % i) for label in data.columns]
    
    # Combine
    agg = pd.concat(cols, axis=1)
    agg.columns = names
    # Drop rows with NaN values
    if dropnan:
        agg.dropna(inplace=True)
    
    global column_names
    column_names = names

    return agg

scalers = []
for var in range(len(pred_var)):
    scalers.append(MinMaxScaler(feature_range=(pv_range[var][0], pv_range[var][1])))
    scalers[var].fit(np.transpose([[0, 1]]))

def predict_next(_dataset:DataFrame):
    dataset = _dataset.copy()
    # Compute outputs from discrete motor controllers
    motor = np.empty((len(pred_var), numHA, len(dataset.index)))
    for ha in range(numHA):
        for i in range(1, len(dataset.index)):
            for var in range(len(pred_var)):
                motor[var][ha][i] = motor_model(ha, dataset.iloc[i], dataset.iloc[i-1])[var]

    # Drop irrelevant variables
    for column in dataset:
        if vars_used.count(column) == 0:
            dataset.drop(column, axis=1, inplace=True)
        else:
            dataset[column] = pd.to_numeric(dataset[column])

    # Append motor controller outputs
    for var in range(len(pred_var)):
        for ha in range(numHA):
            dataset["controller-" + pred_var[var] + "-" + str(ha)] = motor[var][ha]

    # Move predicted variables to the end for readability
    for var in range(len(pred_var)):
        column_to_move = dataset.pop(pred_var[var])
        dataset.insert(len(dataset.columns), pred_var[var], column_to_move)
    
    dataset = dataset.tail(-1)
    dataset = series_to_supervised(dataset, 1, 1)

    scaler_validation = MinMaxScaler(feature_range=(0, 1))
    for col in dataset.columns:
        # Normalize columns with the same metric using the same scaler
        is_la = False
        for var in range(len(pred_var)):
            if col.startswith("controller-" + pred_var[var] + "-") or col.startswith(pred_var[var]):
                scaler_validation.fit(np.transpose([pv_range[var]]))
                is_la = True
        
        if not is_la:
            scaler_validation.fit(np.transpose([dataset[col]]))
        
        if not col.startswith("HA"):
            dataset[col] = np.transpose(scaler_validation.transform(np.transpose([dataset[col]])))[0]
        
    dataset = dataset.loc[:, ~dataset.columns.str.startswith('HA')]

    validation_X = DataFrame()
    validation_Y = DataFrame()

    # Outputs: predicted variables and discrete motor controller outputs (for use in the loss function)
    for var in range(len(pred_var)):
        validation_Y[pred_var[var] + "(t)"] = dataset[pred_var[var] + "(t)"]

    # Inputs: everything else
    for col in dataset:
        if not col in validation_Y.columns:
            validation_X[col] = dataset[col]

    validation_X = validation_X.to_numpy()
    validation_Y = validation_Y.to_numpy()

    validation_X = validation_X.reshape((validation_X.shape[0], 1, validation_X.shape[1]))

    yhat = model.predict(validation_X)
    la = yhat[len(yhat)-1]
    la = [scalers[i].transform([[la[i]]])[0][0] for i in range(len(la))]
    return la





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

# modified from https://github.com/eleurent/highway-env/blob/31881fbe45fd05dbd3203bb35419ff5fb1b7bc09/highway_env/vehicle/controller.py
# in this version, no extra latent state is stored (target_lane, target_speed)
TURN_HEADING = 0.15 # Target heading when turning
TURN_TARGET = 30 # How much to adjust when targeting a lane (higher = smoother)
max_velocity = 45 # Maximum velocity

last_la = {"steering": 0, "acceleration": 0 }

def run_la(self, action: Union[dict, str] = None, step = True, closest = None) -> None:
    global last_la

    if step:
        Vehicle.act(self, last_la)
        return last_la
    return last_la

ControlledVehicle.act = run_la


dataset_base = pd.read_csv("temp.csv")
success = 0
for iter in range(100):
    obs, info = env.reset()
    dataset = dataset_base.copy(deep=True)

    action = [0, 0]
    for _ in range(150):
        obs, reward, done, truncated, info = env.step(0)

        # Pre-process observations
        lane_class = classifyLane(obs)
        closest = closestVehicles(obs, lane_class)

        obs_pruned = []
        for prop in obs[0][1:]:
            obs_pruned.append(round(prop, 3))
        
        # Nearby vehicles
        for v in closest:
            for prop in v[1:]:
                obs_pruned.append(round(prop, 3))

        obs_pruned += action + [0]
        dataset.loc[len(dataset)] = obs_pruned

        action = predict_next(dataset)
        last_la = {"steering": action[0], "acceleration": action[1]}

    if(obs[0][3] > 10 and obs[0][2] > -4 and obs[0][2] < 25):
        success += 1
        print(success)
        
print(success)
env.close()