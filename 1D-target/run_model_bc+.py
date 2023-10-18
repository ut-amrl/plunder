from gym.envs.registration import register
import gym
import csv
import numpy as np
import sys
import time
import random
import pandas as pd
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

training_set = 10
validation_set = 30 # including training_set
train_time = 10000
patience = 500
sim_time = 126
samples = 100
folder = "../baselines/supervised_learning_ha/target/"
vars_used = [
    "HA",
    "pos",
    "decMax",
    "accMax",
    "vel",
    "vMax",
    "LA.acc",
]
pred_var = ["LA.acc"]
pv_range = [[-50, 50]]
pv_stddev = [0.5]

numHA = 3
def motor_model(ha, data, data_prev):
    if ha == 0:
        return [min(data_prev["LA.acc"] + 1, data["accMax"])]
    elif ha == 1:
        if data_prev["LA.acc"] < 0:
            return [min(data_prev["LA.acc"] + 1, 0)]
        elif data_prev["LA.acc"] >= 0:
            return [max(data_prev["LA.acc"] - 1, 0)]
    else:
        return [max(data_prev["LA.acc"] - 1, data["decMax"])]



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

model = keras.models.load_model(folder + "model_ss", compile=False)
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






sys.path.append("../baselines/gail/")
env_id = "env-1d-v1"
register(id=env_id, entry_point='custom_envs.envs:Env_1d')

env = gym.make(env_id)

dataset_base = pd.read_csv("temp.csv")
success = 0
for _ in range(100):
    env.config(random.randint(-10, -5), random.randint(5, 10), random.randint(5, 30), 100)
    observation = env.reset()
    dataset = dataset_base.copy(deep=True)

    action = [0]

    for t in range(1000):
        observation, _, _, _ = env.step(action)

        pos, decMax, accMax, vMax, vel, acc, ACC, CON, DEC = observation[0], observation[1], observation[2], observation[3], observation[4], observation[5], observation[6], observation[7], observation[8]
        target = 100

        obs_pruned = [t, acc, pos, decMax, accMax, vMax, vel, target, target - pos, 0]
        dataset.loc[len(dataset)] = obs_pruned
        action = predict_next(dataset)
        
        if vel < -10:
            break
        if pos > 200:
            break
        if pos < 140 and pos > 60 and abs(vel) < 0.1:
            success += 1
            break
    
    print(observation)

print(success)
env.close()