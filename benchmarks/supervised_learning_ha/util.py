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

def obs_likelihood(ha, data, data_prev):
    mean = settings.motor_model(ha, data, data_prev)
    obs_log = 0
    if not settings.pred_var1 == None:
        obs_log += norm(mean[0], settings.pv1_stddev).logpdf(data[settings.pred_var1])
    if not settings.pred_var2 == None:
        obs_log += norm(mean[1], settings.pv2_stddev).logpdf(data[settings.pred_var2])

    return obs_log

def gen_traj(y_pred, y_true):
    la1 = []
    la2 = []
    for i in range(len(y_pred)):
        # Perform a weighted sum of all controllers
        # Output 1
        if not settings.pred_var1 == None:
            la1.append(tensorflow.reduce_sum(tensorflow.multiply(y_pred, y_true[:, 1:settings.numHA+1]), 1))

        # Output 2
        if not settings.pred_var2 == None:
            la2.append(tensorflow.reduce_sum(tensorflow.multiply(y_pred, y_true[:, settings.numHA+2:]), 1))
    
    return (la1, la2)

scaler1 = MinMaxScaler(feature_range=(settings.pv1_range[0], settings.pv1_range[1]))
scaler1.fit(np.transpose([[0, 1]]))

scaler2 = MinMaxScaler(feature_range=(settings.pv2_range[0], settings.pv2_range[1]))
scaler2.fit(np.transpose([[0, 1]]))

def log_obs(expected, actual, var1):
    if var1:
        mean = scaler1.transform([[expected]])[0][0]
        test = scaler1.transform([[actual]])[0][0]
        obs_log += norm(mean, settings.pv1_stddev).logpdf(test)
    else:
        mean = scaler2.transform([[expected]])[0][0]
        test = scaler2.transform([[actual]])[0][0]
        obs_log += norm(mean, settings.pv2_stddev).logpdf(test)

    return obs_log

def cum_log_obs(expected1, expected2, actual):
    error = 0
    for i in range(len(actual)):
        # Output 1
        if not settings.pred_var1 == None:
            error += log_obs(expected1[i], y_true[:, :1], True)  

        # Output 2
        if not settings.pred_var2 == None:
            error += log_obs(expected2[i], y_true[:, settings.numHA+1:settings.numHA+2], False)
        
    return error / len(expected)
    
