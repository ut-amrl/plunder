import pandas as pd
import numpy as np

# Setting: 1D-target
training_set = 10
validation_set = 10 # including training_set
train_time = 10000
folder = "target-sim/"
vars_used = [
    "pos",
    "decMax",
    "accMax",
    "vel",
    "vMax",
    "LA.acc",
]
pred_var1 = "LA.acc"
pv1_range = [-50, 50]
pred_var2 = None
pv2_range = None

numHA = 3
def motor_model(ha, data, data_prev):
    if ha == 0:
        return (min(data_prev["LA.acc"] + 1, data["accMax"]), None)
    elif ha == 1:
        if data["LA.acc"] < 0:
            return (min(data_prev["LA.acc"] + 1, data["accMax"]), None)
        elif data["LA.acc"] > 0:
            return (max(data_prev["LA.acc"] - 1, data["decMax"]), None)
    else:
        return (max(data_prev["LA.acc"] - 1, data["decMax"]), None)



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
# pred_var2 = None

# numHA = 3
# def motor_model(ha, data, data_prev):
    # pass

# Setting: 2D-highway-env
# training_set = 10
# validation_set = 20 # including training_set
# train_time = 3000
# folder = "highway-sim/"
# vars_used = [
#     "l_x",
#     "f_x",
#     "r_x",
#     "LA.steer",
#     "LA.acc"
# ]
# pred_var1 = "LA.steer"
# pred_var2 = "LA.acc"

# numHA = 4
# def motor_model(ha, data, data_prev):
    # pass