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

def gen_traj(y_pred, y_true):
    la1, la2 = None, None

    # Perform a weighted sum of all controllers
    # Output 1
    if not settings.pred_var1 == None:
        la1 = tensorflow.reduce_sum(tensorflow.multiply(y_pred, y_true[:, 1:settings.numHA+1]), 1)

    # Output 2
    if not settings.pred_var2 == None:
        la2 = tensorflow.reduce_sum(tensorflow.multiply(y_pred, y_true[:, settings.numHA+2:]), 1)
    
    return (la1, la2)

# def log_obs(expected, actual, var1):
#     obs_log = 0
    
#     if var1:
#         z_score = (expected - actual) / settings.pv1_stddev
#         obs_log += norm(0, 1).logpdf(z_score)
#     else:
#         z_score = (expected - actual) / settings.pv2_stddev
#         obs_log += norm(0, 1).logpdf(z_score)
    
#     return obs_log

def log_obs(expected, actual, var1, scaler):
    obs_log = 0
    mean = scaler.transform([[expected]])[0][0]
    test = scaler.transform([[actual]])[0][0]
    
    if var1:
        obs_log += norm(mean, settings.pv1_stddev).logpdf(test)
    else:
        obs_log += norm(mean, settings.pv2_stddev).logpdf(test)
    
    return obs_log

# def cum_log_obs(expected1, expected2, actual):
#     if not settings.pred_var1 == None:
#         actual1 = actual[:, 0]
#     if not settings.pred_var2 == None:
#         actual2 = actual[:, 1]

#     error = 0
#     for i in range(len(actual)):
#         # Output 1
#         if not settings.pred_var1 == None:
#             error += log_obs(expected1[i], actual1[i], True)  

#         # Output 2
#         if not settings.pred_var2 == None:
#             error += log_obs(expected2[i], actual2[i], False)
        
#     return error / len(actual)
def cum_log_obs(expected1, expected2, actual):
    if not settings.pred_var1 == None:
        actual1 = actual[:, 0]
    if not settings.pred_var2 == None:
        actual2 = actual[:, 1]

    if not settings.pred_var1 == None:
        scaler1 = MinMaxScaler(feature_range=(settings.pv1_range[0], settings.pv1_range[1]))
        scaler1.fit(np.transpose([[0, 1]]))

    if not settings.pred_var2 == None:
        scaler2 = MinMaxScaler(feature_range=(settings.pv2_range[0], settings.pv2_range[1]))
        scaler2.fit(np.transpose([[0, 1]]))
    
    error = 0
    for i in range(len(actual)):
        # Output 1
        if not settings.pred_var1 == None:
            error += log_obs(expected1[i], actual1[i], True, scaler1)  

        # Output 2
        if not settings.pred_var2 == None:
            error += log_obs(expected2[i], actual2[i], False, scaler2)
        
    return error / len(actual)