import pandas as pd
import numpy as np

# Setting: 1D-target
# training_set = 10
# validation_set = 30 # including training_set
# train_time = 10000
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
# pred_var = ["LA.acc"]
# pv_range = [[-50, 50]]
# pv_stddev = [0.5]

# numHA = 3
# def motor_model(ha, data, data_prev):
#     if ha == 0:
#         return [min(data_prev["LA.acc"] + 1, data["accMax"])]
#     elif ha == 1:
#         if data_prev["LA.acc"] < 0:
#             return [min(data_prev["LA.acc"] + 1, data["accMax"])]
#         elif data_prev["LA.acc"] > 0:
#             return [max(data_prev["LA.acc"] - 1, data["decMax"])]
#     else:
#         return [max(data_prev["LA.acc"] - 1, data["decMax"])]

# Setting: 2D-highway-env
# training_set = 10
# validation_set = 30 # including training_set
# train_time = 15000
# patience = 800
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
# pred_var = ["LA.steer", "LA.acc"]
# pv_range = [
#     [-0.3, 0.3],
#     [-30, 30]
# ]
# pv_stddev = [0.01, 2]

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

#     return [target_steer, target_acc]


# Setting: 2D-merge
# training_set = 10
# validation_set = 30 # including training_set
# train_time = 15000
# patience = 100
# sim_time = 75
# samples = 50
# # folder = "merge-easy-data/"
# # folder = "merge-medium-data/"
# # folder = "merge-hard-data/"
# folder = "merge-impossible-data/"
# vars_used = [
#     "HA",
#     "x",
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
# pred_var = ["LA.steer", "LA.acc"]
# pv_range = [
#     [-0.3, 0.3],
#     [-30, 30]
# ]
# pv_stddev = [0.03, 3]

# numHA = 4

# TURN_HEADING = 0.15 # Target heading when turning
# TURN_TARGET = 30 # How much to adjust when targeting a lane (higher = smoother)
# max_velocity = 45 # Maximum velocity

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
#         target_acc = -0.5
#         target_heading = -TURN_HEADING
#     else:
#         target_acc = -0.5
#         target_heading = TURN_HEADING

#     target_steer = target_heading - data["heading"]

#     if target_steer > data_prev["LA.steer"]:
#         target_steer = min(target_steer, data_prev["LA.steer"] + 0.08)
#     else:
#         target_steer = max(target_steer, data_prev["LA.steer"] - 0.08)

#     if target_acc > data_prev["LA.acc"]:
#         target_acc = min(target_acc, data_prev["LA.acc"] + 4)
#     else:
#         target_acc = max(target_acc, data_prev["LA.acc"] - 6)

#     return [target_steer, target_acc]

# Setting: panda-pick-place
training_set = 5
validation_set = 20 # including training_set
train_time = 10000
patience = 0
sim_time = 50
samples = 50
# folder = "panda-pick-place/"
folder = "panda-pick-place-policy/"
vars_used = [
    "HA",
    "x",
    "y",
    "z",
    "bx",
    "by",
    "bz",
    "tx",
    "ty",
    "tz",
    "LA.vx",
    "LA.vy",
    "LA.vz",
    "LA.end"
]
pred_var = ["LA.vx", "LA.vy", "LA.vz", "LA.end"]
pv_range = [
    [-4, 4],
    [-4, 4],
    [-4, 4],
    [-4, 4]
]
# pv_stddev = [0.5, 0.5, 0.5, 0.5]
pv_stddev = [0.2, 0.2, 0.2, 0.2]

numHA = 2

# def motor_model(ha, data, data_prev):
#     if ha == 0:
#         vx = 5 * (data["bx"] - data["x"])
#         vy = 5 * (data["by"] - data["y"])
#         vz = 5 * (data["bz"] - data["z"])
#         end = 1
#     else:
#         vx = 5 * (data["tx"] - data["x"])
#         vy = 5 * (data["ty"] - data["y"])
#         vz = 5 * (data["tz"] - data["z"])
#         end = -1
    
#     return [vx, vy, vz, end]

def motor_model(ha, data, data_prev):
    bx, by, bz = data["bx"] - data["x"], data["by"] - data["y"], data["bz"] - data["z"]
    
    if ha == 0:
        return [bx * 5.0, by * 5.0, bz * 5.0, 0.6]
    
    if data_prev["LA.end"] >= 0.5:
        return [bx * 5.0, by * 5.0, bz * 5.0, 0.3]
    elif data_prev["LA.end"] >= 0.3:
        return [bx * 5.0, by * 5.0, bz * 5.0, 0]
    elif data_prev["LA.end"] >= 0.1:
        return [bx * 5.0, by * 5.0, bz * 5.0, -0.3]
    elif data_prev["LA.end"] >= -0.1:
        return [bx * 5.0, by * 5.0, bz * 5.0, -0.6]
    
    vx = 5 * (data["tx"] - data["x"])
    vy = 5 * (data["ty"] - data["y"])
    vz = 5 * (data["tz"] - data["z"])
    end = -0.6
    
    return [vx, vy, vz, end]



# Setting: panda-stack
# training_set = 10
# validation_set = 20 # including training_set
# train_time = 15000
# patience = 500
# sim_time = 150
# samples = 50
# folder = "panda-stack/"
# vars_used = [
#     "HA",
#     "x",
#     "y",
#     "z",
#     "bx1",
#     "by1",
#     "bz1",
#     "bx2",
#     "by2"
#     "bz2",
#     "tx2",
#     "ty2",
#     "tz2",
#     "LA.vx",
#     "LA.vy",
#     "LA.vz",
#     "LA.end"
# ]
# pred_var = ["LA.vx", "LA.vy", "LA.vz", "LA.end"]
# pv_range = [
#     [-4, 4],
#     [-4, 4],
#     [-4, 4],
#     [-4, 4]
# ]
# pv_stddev = [0.3, 0.3, 0.3, 0.3]

# numHA = 5

# def bound(x):
#     return min(max(x, -4), 4)

# def motor_model(ha, data, data_prev):
#     bx1, by1, bz1, bx2, by2, bz2 = data["bx1"], data["by1"], data["bz1"], data["bx2"], data["by2"], data["bz2"]
#     tx2, ty2, tz2 = data["tx2"], data["ty2"], data["tz2"]

#     tz2 += 0.01

#     if ha == 0:
#         action = [bx1 * 4.0, by1 * 4.0, bz1 * 4.0, 1]
#     elif ha == 1:
#         action = [tx2 * 4.0, ty2 * 4.0, tz2 * 4.0, -1]
#         if action[2] < 0:
#             action[2] = max(data_prev["LA.vz"] - 0.15, action[2])
#     elif ha == 2:
#         action = [0, 0, 0.5, 1]
#     elif ha == 3:
#         action = [bx2 * 4.0, by2 * 4.0, bz2 * 4.0, 1]
#         if action[2] < 0:
#             action[2] = max(data_prev["LA.vz"] - 0.15, action[2])
#     elif ha == 4:
#         action = [0, 0, 0.5, -1]

#     # action = [bound(i) for i in action]
#     return action