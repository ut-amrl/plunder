import pandas as pd
import numpy as np

# Setting: 1D-target
# training_set = 10
# validation_set = 30 # including training_set
# train_time = 20000
# sim_time = 126
# samples = 100
# # folder = "target-easy/"
# # folder = "target-medium/"
# folder = "target-hard/"
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
# # pv1_stddev = 0.3
# # pv1_stddev = 0.6
# pv1_stddev = 1
# pred_var2 = None
# pv2_range = None
# pv2_stddev = None

# numHA = 3
# def motor_model(ha, data, data_prev):
#     if ha == 0:
#         return (min(data_prev["LA.acc"] + 1, data["accMax"]), None)
#     elif ha == 1:
#         if data["LA.acc"] < 0:
#             return (min(data_prev["LA.acc"] + 1, data["accMax"]), None)
#         elif data["LA.acc"] > 0:
#             return (max(data_prev["LA.acc"] - 1, data["decMax"]), None)
#     else:
#         return (max(data_prev["LA.acc"] - 1, data["decMax"]), None)


# Setting: 2D-merge
# training_set = 10
# validation_set = 20 # including training_set
# train_time = 3000
# folder = "merge-sim/"
# vars_used = [
#     "y",
#     "f_x",
#     "r_x",
#     "l_x",
#     "LA.steer"
# ]
# pred_var1 = "LA.steer"
# pv1_range = [-0.1, 0.1]
# pred_var2 = None
# pv2_range = None

# numHA = 3
# KP_H = 0.4 # Turning rate
# TURN_HEADING = 0.2 # Target heading when turning
# TURN_TARGET = 30 # How much to adjust when targeting a lane (higher = smoother)

# def laneFinder(y):
#     return round(y / 4)

# def motor_model(ha, data, data_prev):
#     target_heading = 0.0
#     if ha == 0:
#         target_y = laneFinder(data["y"]) * 4
#         target_heading = np.arctan((target_y - data["y"]) / TURN_TARGET)
#     elif ha == 1:
#         target_heading = -TURN_HEADING
#     else:
#         target_heading = TURN_HEADING

#     return ((target_heading - data["heading"]) * KP_H, None)

# Setting: 2D-highway-env
training_set = 6
validation_set = 20 # including training_set
train_time = 15000
sim_time = 200
samples = 50
# folder = "highway-easy/"
folder = "highway-medium/"
# folder = "highway-hard/"
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
pred_var1 = "LA.steer"
pv1_range = [-0.3, 0.3]
# pv1_stddev = 0.001
pv1_stddev = 0.015
# pv1_stddev = 0.03
pred_var2 = "LA.acc"
pv2_range = [-12, 12]
# pv2_stddev = 0.1
pv2_stddev = 1
# pv2_stddev = 2

numHA = 4

KP_A = 0.4 # Jerk constant (higher = faster acceleration)
KP_H = 0.4 # Turning rate
TURN_HEADING = 0.2 # Target heading when turning
TURN_TARGET = 30 # How much to adjust when targeting a lane (higher = smoother)
min_velocity = 16 # Minimum velocity
max_velocity = 30 # Maximum velocity

def laneFinder(y):
    return round(y / 4)

def motor_model(ha, data, data_prev):
    acc = 0.0
    target_heading = 0.0
    if ha == 0:
        acc = KP_A * (max_velocity - data["vx"])

        target_y = laneFinder(data["y"]) * 4
        target_heading = np.arctan((target_y - data["y"]) / TURN_TARGET)
    elif ha == 1:
        acc = KP_A * (min_velocity - data["vx"])

        target_y = laneFinder(data["y"]) * 4
        target_heading = np.arctan((target_y - data["y"]) / TURN_TARGET)
    elif ha == 2:
        target_heading = -TURN_HEADING
    else:
        target_heading = TURN_HEADING

    return ((target_heading - data["heading"]) * KP_H, acc)
