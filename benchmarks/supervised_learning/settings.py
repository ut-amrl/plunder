import json
import pandas as pd
import numpy as np

# Setting: 1D-target
training_set = 5
validation_set = 10 # including training_set
folder = "sim/"
vars_used = [
    "LA.acc",
    "pos",
    "decMax",
    "accMax",
    "vel",
    "vMax"
]
pred_vars = [
    "LA.acc"
]

# Setting: 2D-merge

# Setting: 2D-highway-env