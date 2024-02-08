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

training_set = 5
validation_set = 10 # including training_set
train_time = 12000
patience = 500
sim_time = 30
samples = 50
folder = "../../baselines/supervised_learning_ha/panda-pick-place-rl/"
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
pv_stddev = [0.5, 0.5, 0.5, 0.5]

numHA = 2

def return_closer(next, cur):
    if next > cur:
        return min(next, cur + 0.4)
    return max(next, cur - 0.4)

def motor_model(ha, data, data_prev):
    bx, by, bz = data["bx"] - data["x"], data["by"] - data["y"], data["bz"] - data["z"]
    tx, ty, tz = data["tx"] - data["x"], data["ty"] - data["y"], data["tz"] - data["z"]

    if ha == 0:
        res = [bx * 5.0, by * 5.0, bz * 5.0, 1]
    else:
        res = [tx * 5.0, ty * 5.0, tz * 5.0, -1]
    
    res[0] = return_closer(res[0], data_prev["LA.vx"])
    res[1] = return_closer(res[1], data_prev["LA.vy"])
    res[2] = return_closer(res[2], data_prev["LA.vz"])

    return res


# Load model

# Custom metric
mse = tensorflow.keras.losses.MeanSquaredError()
class CustomAccuracy(keras.losses.Loss):
    def __init__(self):
        super().__init__()
    def call(self, y_true, y_pred):
        # Perform a weighted sum of all controllers
        error = 0

        idx = 1
        for var in range(len(pred_var)):
            sum = tensorflow.reduce_sum(tensorflow.multiply(y_pred, y_true[:, idx:idx+numHA]), 1)
            error += mse(sum, y_true[:, idx-1])
            idx += numHA + 1

        return error

model = keras.models.load_model(folder + "model_pprl", compile=False)
model.compile(loss=CustomAccuracy(), optimizer='adam')


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
        for ha in range(numHA):
            validation_Y["controller-" + pred_var[var] + "-" + str(ha) + "(t)"] = dataset["controller-" + pred_var[var] + "-" + str(ha) + "(t)"]
    
    # Inputs: everything else
    for col in dataset:
        if not col in validation_Y.columns:
            validation_X[col] = dataset[col]

    validation_X = validation_X.to_numpy()
    validation_Y = validation_Y.to_numpy()

    validation_X = validation_X.reshape((validation_X.shape[0], 1, validation_X.shape[1]))

    yhat = model.predict(validation_X)
    yhat = yhat[len(yhat)-1]

    la = []
    # Perform a weighted sum of all controllers
    idx = 1
    for var in range(len(pred_var)):
        sum = np.sum(np.multiply(yhat, validation_Y[len(validation_Y)-1, idx:idx+numHA]))
        la.append(sum)
        idx += numHA + 1

    la = [scalers[i].transform([[la[i]]])[0][0] for i in range(len(la))]

    return la






env = gym.make("PandaPickAndPlace-v3", render_mode="human")
dataset_base = pd.read_csv("temp.csv")

success = 0
for iter in range(100):
    observation, info = env.reset()
    dataset = dataset_base.copy(deep=True)

    action = [0, 0, 0, 0]
    for _ in range(30):
        observation, reward, terminated, truncated, info = env.step(action)

        world_state = observation["observation"]
        target_pos = observation["desired_goal"][0:3]

        x, y, z, bx, by, bz, tx, ty, tz, end_width = world_state[0], world_state[1], world_state[2], world_state[7], world_state[8], world_state[9], target_pos[0], target_pos[1], target_pos[2], world_state[6]

        obs_pruned = [x, y, z, bx, by, bz, tx, ty, tz, end_width] + action + [0]
        dataset.loc[len(dataset)] = obs_pruned

        action = predict_next(dataset)

        if terminated:
            success += 1
            break
        
print(success)
env.close()