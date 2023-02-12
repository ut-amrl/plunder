import json
import pandas as pd
import numpy as np

# Setting: 1D-target
training_set = 9
validation_set = 10 # including training_set
train_time = 3000
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
pred_var2 = None

numHA = 3
def motor_model(ha, data):
    if ha == 0:
        return (min(data["LA.acc"] + 1, data["accMax"]), None)
    elif ha == 1:
        if data["LA.acc"] < 0:
            return (min(data["LA.acc"] + 1, data["accMax"]), None)
        elif data["LA.acc"] > 0:
            return (max(data["LA.acc"] - 1, data["decMax"]), None)
    else:
        return (max(data["LA.acc"] - 1, data["decMax"]), None)



# Setting: 2D-merge
# training_set = 10
# validation_set = 20 # including training_set
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

# Setting: 2D-highway-env
# training_set = 10
# validation_set = 20 # including training_set
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