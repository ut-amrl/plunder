import pandas as pd
import numpy as np

# Setting: 1D-target
# training_set = 10
# validation_set = 30 # including training_set
# train_time = 20000
# patience = 500
# sim_time = 126
# samples = 100
# folder = "target/"
# vars_used = [
#     "HA",
#     "pos",
#     "decMax",
#     "accMax",
#     "vel",
#     "vMax",
#     "LA.acc",
# ]
# pred_var1 = "LA.acc"
# pv1_range = [-50, 50]
# pv1_stddev = 0.5
# pred_var2 = None
# pv2_range = None
# pv2_stddev = None

# numHA = 3
# def motor_model(ha, data, data_prev):
#     if ha == 0:
#         return (min(data_prev["LA.acc"] + 1, data["accMax"]), None)
#     elif ha == 1:
#         if data_prev["LA.acc"] < 0:
#             return (min(data_prev["LA.acc"] + 1, data["accMax"]), None)
#         elif data_prev["LA.acc"] > 0:
#             return (max(data_prev["LA.acc"] - 1, data["decMax"]), None)
#     else:
#         return (max(data_prev["LA.acc"] - 1, data["decMax"]), None)

# Setting: 2D-highway-env
# training_set = 10
# validation_set = 30 # including training_set
# train_time = 15000
# patience = 5000
# sim_time = 150
# samples = 50
# folder = "highway/"
# vars_used = [
#     "HA",
#     "x",
#     "y",
#     "l_x",
#     "f_x",
#     "r_x",
#     "v_x",
#     "l_vx",
#     "f_vx",
#     "r_vx",
#     "LA.steer",
#     "LA.acc"
# ]
# pred_var1 = "LA.steer"
# pv1_range = [-0.3, 0.3]
# pv1_stddev = 0.01
# pred_var2 = "LA.acc"
# pv2_range = [-30, 30]
# pv2_stddev = 2

# numHA = 4

# KP_H = 0.5 # Turning rate
# TURN_HEADING = 0.15 # Target heading when turning
# TURN_TARGET = 30 # How much to adjust when targeting a lane (higher = smoother)
# max_velocity = 40 # Maximum velocity
# turn_velocity = 30 # Turning velocity

# def laneFinder(y):
#     return round(y / 4)

# def motor_model(ha, data, data_prev):
#     target_acc = 0.0
#     target_heading = 0.0
#     if ha == 0:
#         target_acc = max_velocity - data["vx"]

#         target_y = laneFinder(data["y"]) * 4
#         target_heading = np.arctan((target_y - data["y"]) / TURN_TARGET)
#     elif ha == 1:
#         target_acc = data["f_vx"] - data["vx"]

#         target_y = laneFinder(data["y"]) * 4
#         target_heading = np.arctan((target_y - data["y"]) / TURN_TARGET)
#     elif ha == 2:
#         target_acc = turn_velocity - data["vx"]
#         target_heading = -TURN_HEADING
#     else:
#         target_acc = turn_velocity - data["vx"]
#         target_heading = TURN_HEADING

#     target_steer = target_heading - data["heading"]
#     if(target_steer > data_prev["LA.steer"]):
#         target_steer = min(target_steer, data_prev["LA.steer"] + 0.04)
#     else:
#         target_steer = max(target_steer, data_prev["LA.steer"] - 0.04)

#     if(target_acc > data_prev["LA.acc"]):  
#         target_acc = min(target_acc, data_prev["LA.acc"] + 4)
#     else:
#         target_acc = max(target_acc, data_prev["LA.acc"] - 6)

#     return (target_steer, target_acc)

# Setting: 2D-merge
training_set = 10
validation_set = 30 # including training_set
train_time = 15000
patience = 500
sim_time = 75
samples = 50
folder = "merge-easy-data/"
# folder = "merge-medium-data/"
vars_used = [
    "HA",
    "x",
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
pred_var1 = "LA.steer"
pv1_range = [-0.3, 0.3]
pv1_stddev = 0.03
pred_var2 = "LA.acc"
pv2_range = [-30, 30]
pv2_stddev = 3

numHA = 4

TURN_HEADING = 0.15 # Target heading when turning
TURN_TARGET = 30 # How much to adjust when targeting a lane (higher = smoother)
max_velocity = 45 # Maximum velocity

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
        target_acc = -0.5
        target_heading = -TURN_HEADING
    else:
        target_acc = -0.5
        target_heading = TURN_HEADING

    target_steer = target_heading - data["heading"]

    if target_steer > data_prev["LA.steer"]:
        target_steer = min(target_steer, data_prev["LA.steer"] + 0.08)
    else:
        target_steer = max(target_steer, data_prev["LA.steer"] - 0.08)

    if target_acc > data_prev["LA.acc"]:
        target_acc = min(target_acc, data_prev["LA.acc"] + 4)
    else:
        target_acc = max(target_acc, data_prev["LA.acc"] - 6)

    return (target_steer, target_acc)
