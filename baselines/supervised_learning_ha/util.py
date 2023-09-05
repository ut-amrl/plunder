import numpy as np
import pandas as pd
import tensorflow
from tensorflow import keras
from keras.models import Sequential
from keras.layers import Dense, LSTM
import math
from matplotlib import pyplot
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from matplotlib.figure import Figure
from pandas import DataFrame
from sklearn.preprocessing import MinMaxScaler, LabelEncoder
from scipy.stats import norm

import settings

def percent_accuracy(yhat_test, data):
    total_ha = 0
    correct_ha = 0
    for i in range(len(yhat_test)):
        total_ha += 1
        correct_ha += yhat_test[i][round(data["HA(t)"][i])]
    
    return correct_ha / total_ha * 100

def gen_traj(y_pred, y_true):
    la = []

    # Perform a weighted sum of all controllers
    idx = 1
    for var in range(len(settings.pred_var)):
        sum = tensorflow.reduce_sum(tensorflow.multiply(y_pred, y_true[:, idx:idx+settings.numHA]), 1)
        la.append(sum)
        idx += settings.numHA + 1

    return la

def log_obs(expected, actual, var, scaler):
    obs_log = 0
    mean = scaler.transform([[expected]])[0][0]
    test = scaler.transform([[actual]])[0][0]
    
    obs_log += norm(mean, settings.pv_stddev[var]).logpdf(test)
    return obs_log

def cum_log_obs(expected, actual_transposed):
    actual = []
    for var in range(len(settings.pred_var)):
        actual.append(actual_transposed[:, var])
    
    scalers = []
    for var in range(len(settings.pred_var)):
        scalers.append(MinMaxScaler(feature_range=(settings.pv_range[var][0], settings.pv_range[var][1])))
        scalers[var].fit(np.transpose([[0, 1]]))
    
    error = 0
    for i in range(len(actual_transposed)):
        for var in range(len(settings.pred_var)):
            error += log_obs(expected[var][i], actual[var][i], var, scalers[var])
        
    return error / len(actual_transposed)
