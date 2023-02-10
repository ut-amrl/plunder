import json
import pandas as pd
import numpy as np

# Setting: 1D-target
# training_set = 5
# validation_set = 10 # including training_set
# folder = "target-sim/"
# vars_used = [
#     "pos",
#     "decMax",
#     "accMax",
#     "vel",
#     "vMax",
#     "LA.acc",
# ]
# pred_var1 = "LA.acc"
# pred_var2 = None

# Setting: 2D-merge
training_set = 5
validation_set = 20 # including training_set
folder = "merge-sim/"
vars_used = [
    "y",
    "f_x",
    "r_x",
    "l_x",
    "LA.steer" # make sure this is last
]
pred_var1 = "LA.steer"
pred_var2 = None

# Setting: 2D-highway-env