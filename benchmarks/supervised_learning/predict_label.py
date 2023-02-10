# https://machinelearningmastery.com/multivariate-time-series-forecasting-lstms-keras/
import numpy as np
import pandas as pd
# Requires all tensorflow dependencies
try:
    from tensorflow import keras
    # import tensorflow.keras as keras
except:
    print("Error: Tensorflow import failed")
    exit(0)

# import datetime
from datetime import *
import math
from math import sqrt
from numpy import concatenate
from matplotlib import pyplot
from pandas import read_csv
from pandas import DataFrame
from pandas import concat
from sklearn.preprocessing import MinMaxScaler
from sklearn.preprocessing import LabelEncoder
from sklearn.metrics import mean_squared_error
from keras.models import Sequential
from keras.layers import Dense
from keras.layers import LSTM

from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from matplotlib.figure import Figure
import settings

column_names = []

# Convert series to supervised learning
def series_to_supervised(data, n_in=4, n_out=1, dropnan=True):
    n_vars = 1 if type(data) is list else data.shape[1]
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
    agg = concat(cols, axis=1)
    agg.columns = names
    # Drop rows with NaN values
    if dropnan:
        agg.dropna(inplace=True)
    
    global column_names 
    column_names = names

    return agg

fig_test = Figure()
fig_valid = Figure()

figs = []  # fig_test, fig_valid
results = []  # datetime, yhat_valid, y_validation
toReturn = []

def loadDataFrame():

    dataset_validation = DataFrame()

    training_size = 0

    for iter in range(settings.validation_set):
        f = open(settings.folder + "data" + str(iter) + ".csv")
        Lines = f.readlines()
        for i in range(0, len(Lines)):
            Lines[i] = ''.join(Lines[i].split())

        out = open("formatted_data.csv", "w")
        for line in Lines:
            out.write(line + "\n")

        out.close()
        f.close() 

        # Load dataset
        dataset = pd.read_csv("formatted_data.csv")

        # print("Original dataset:")
        # print(dataset)

        for column in dataset:
            if settings.vars_used.count(column) == 0:
                dataset.drop(column, axis=1, inplace=True)
            else:
                dataset[column] = pd.to_numeric(dataset[column])
        
        if not settings.pred_var1 == None:
            column_to_move = dataset.pop(settings.pred_var1)
            dataset.insert(len(dataset.columns), settings.pred_var1, column_to_move)
        
        if not settings.pred_var2 == None:
            column_to_move = dataset.pop(settings.pred_var2)
            dataset.insert(len(dataset.columns), settings.pred_var2, column_to_move)
        
        # print("Relevant Variables:")
        # print(dataset)

        # Convert from series to supervised learning problem
        dataset = series_to_supervised(dataset, 4, 1)

        dataset_validation = pd.concat([dataset_validation, dataset], ignore_index=True)
        if iter < settings.training_set:
            training_size = len(dataset_validation.index)
    
    # print("Relevant data:")
    # print(dataset_validation)

    # Normalize features
    values_validation = dataset_validation.values.astype('float32')
    scaler_validation = MinMaxScaler(feature_range=(0, 1))
    validation_df = DataFrame(scaler_validation.fit_transform(values_validation))
    validation_df.columns = column_names

    training_df = validation_df.loc[0:training_size-1]
    
    print("\n\nRelevant data: Scaled and reframed as a supervised learning problem\n\n")
    print("Validation set:")
    print(validation_df)
    print("Training set:")
    print(training_df)

    # Split into training and validation sets
    makePredictions(training_df, validation_df)


# TODO
def makePredictions(df_train, df_validation):
    scaler = MinMaxScaler(feature_range=(0, 1))

    min_la1 = df_validation[settings.pred_var1+'(t)'].min()
    mean_la1 = df_validation[settings.pred_var1+'(t)'].mean()
    max_la1 = df_validation[settings.pred_var1+'(t)'].max()

    df_la1_levels = pd.DataFrame(
        {settings.pred_var1: [min_la1, mean_la1, max_la1]})

    la1_scaled = scaler.fit_transform(df_la1_levels.values)
    la1_scaled = la1_scaled[1][0]

    la1_levels_scaled = [(1.5 * la1_scaled), (la1_scaled), (
        0.5 * la1_scaled), (0.25 * la1_scaled)]
    la1_colors = ['r', 'tab:orange', 'y', 'g']
    la1_labels = ['150% Mean', 'Mean LA 1', '50% Mean', '25% Mean']

    # Split into train and test sets
    values = df_train.values
    n_train_hours = math.floor(len(df_train.index) * 0.1)
    train = values[n_train_hours:]
    test = values[:n_train_hours]

    # print(df_train)
    print("Training set size")
    print(len(train))
    # print(train)
    print("Test set size")
    print(len(test))
    # print(test)
    print("Validation set size")
    print(len(df_validation.index))
    # print(df_validation)

    # Split into input and outputs
    train_X, train_y = train[:, :-1], train[:, -1]
    test_X, test_y = test[:, :-1], test[:, -1]

    # Reshape input to be 3D [samples, timesteps, features]
    train_X = train_X.reshape((train_X.shape[0], 1, train_X.shape[1]))
    test_X = test_X.reshape((test_X.shape[0], 1, test_X.shape[1]))

    # Repeat for validation data
    valid_vals = df_validation.values
    X_validation, y_validation = valid_vals[:, :-1], valid_vals[:, -1]
    X_validation = X_validation.reshape((X_validation.shape[0], 1, X_validation.shape[1]))

    # Design network
    model = Sequential()
    model.add(LSTM(128, input_shape=(train_X.shape[1], train_X.shape[2])))
    model.add(Dense(64, activation=keras.activations.sigmoid))
    model.add(Dense(16, activation=keras.activations.sigmoid))
    model.add(Dense(1, activation=keras.activations.sigmoid))
    model.compile(loss='mae', optimizer='rmsprop', metrics=['mse', 'mae'])

    print(model.summary())
    # Fit network
    history = model.fit(train_X, train_y, epochs=2000, batch_size=100, validation_data=(test_X, test_y), verbose=2, shuffle=False)  # validation_split= 0.2)

    # Plot history
    pyplot.plot(history.history['loss'], label='train_loss')
    pyplot.plot(history.history['val_loss'], label='validation_loss')
    pyplot.legend()
    # pyplot.savefig("loss.jpg")
    pyplot.show()

    # Make a prediction and plot results
    yhat_test = model.predict(test_X)
    # print("TEST_X")
    # print(test_X.shape)
    # print("YHAT_TEST")
    # print(yhat_test)

    pyplot.plot(test_y, label='test_y')
    pyplot.plot(yhat_test, label='yhat_test')
    for laInd in range(len(la1_levels_scaled)):
        pyplot.axhline(y=la1_levels_scaled[laInd], color=la1_colors[laInd],
                       linestyle='-', label=la1_labels[laInd])
    pyplot.legend()
    # pyplot.savefig("test.png")
    pyplot.show()

    # Plot and evaluate prediction results
    axis_test = fig_test.add_subplot(1, 1, 1)
    axis_test.plot(test_y, label='Actual', linewidth=2)
    axis_test.plot(yhat_test, label='Predicted', linewidth=2.5, alpha=0.6, color='tab:pink')
    for laInd in range(len(la1_levels_scaled)):
        axis_test.axhline(y=la1_levels_scaled[laInd], color=la1_colors[laInd], linestyle='-', label=la1_labels[laInd])

    yhat_valid = model.predict(X_validation)

    pyplot.plot(y_validation, label='y_validation')
    pyplot.plot(yhat_valid, label='yhat_valid')
    for laInd in range(len(la1_levels_scaled)):
        pyplot.axhline(y=la1_levels_scaled[laInd], color=la1_colors[laInd], linestyle='-', label=la1_labels[laInd])
    pyplot.legend()
    pyplot.savefig("validation.png")
    pyplot.show()

    axis_valid = fig_valid.add_subplot(1, 1, 1)
    axis_valid.plot(y_validation, label='Actual', linewidth=2)
    axis_valid.plot(yhat_valid, label='Predicted',
                    linewidth=3, alpha=0.7, color='tab:pink')
    for laInd in range(len(la1_levels_scaled)):
        axis_valid.axhline(y=la1_levels_scaled[laInd], color=la1_colors[laInd], linestyle='-', label=la1_labels[laInd], linewidth=1)

    # df_validation = df_validation[~df_validation.isin(
    #     [np.nan, np.inf, -np.inf]).any(1)]

    global figs
    global results
    global toReturn

    figs.append(fig_test)
    figs.append(fig_valid)

    try:
        i = 0
        dates = []
        resultsYhat = []
        resultsYvalid = []

        while (i < len(yhat_valid)-4):
            max_yhat_valid = max(max(yhat_valid[i][0], yhat_valid[i+1][0]), max(yhat_valid[i+2][0], yhat_valid[i+3][0]))
            max_y_validation = max(max(y_validation[i], y_validation[i+1]), max(y_validation[i+2], y_validation[i+3]))

            dates.append(
                df_validation.iloc[i, df_validation.columns.get_loc('DateTime')])
            resultsYhat.append(max_yhat_valid / la1_levels_scaled[0])
            resultsYvalid.append(max_y_validation / la1_levels_scaled[0])

            i += 4
        results.append(dates)
        results.append(resultsYhat)
        results.append(resultsYvalid)

        toReturn.append(figs)
        toReturn.append(results)

    except:
        print("ERROR OCCURRED IN PROCESSING OF VALIDATION RESULTS")

    X_validation = X_validation.reshape((X_validation.shape[0], X_validation.shape[2]))

    # Invert scaling for forecast
    inv_yhat = concatenate((yhat_valid, X_validation[:, 1:]), axis=1)

    scaler = MinMaxScaler(feature_range=(0, 1)).fit(inv_yhat)

    inv_yhat = scaler.inverse_transform(inv_yhat)
    inv_yhat = inv_yhat[:, 0]

    # Invert scaling for actual
    y_validation = y_validation.reshape((len(y_validation), 1))
    inv_y = concatenate((y_validation, X_validation[:, 1:]), axis=1)

    inv_y = scaler.inverse_transform(inv_y)
    inv_y = inv_y[:, 0]

    # Calculate RMSE
    rmse = sqrt(mean_squared_error(inv_y, inv_yhat))
    print('Test RMSE: %.3f' % rmse)
