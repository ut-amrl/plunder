# https://machinelearningmastery.com/multivariate-time-series-forecasting-lstms-keras/
# Credit to Justina Lam for providing a baseline for a good chunk of this code
import numpy as np
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

import settings
import plotter
import util

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

def loadDataFrame():
    pd.set_option('display.max_columns', None)
    dataset_validation = DataFrame()

    training_size = 0
    for iter in range(settings.validation_set):
        f = open(settings.folder + "data" + str(iter) + ".csv")
        Lines = f.readlines()
        for i in range(0, len(Lines)):
            Lines[i] = ''.join(Lines[i].split())

        out = open("out/formatted_data.csv", "w")
        for line in Lines:
            out.write(line + "\n")

        out.close()
        f.close() 

        # Load dataset
        dataset = pd.read_csv("out/formatted_data.csv")

        # print("Original dataset:")
        # print(dataset)

        # Compute outputs from discrete motor controllers
        motor = np.empty((len(settings.pred_var), settings.numHA, len(dataset.index)))
        for ha in range(settings.numHA):
            for i in range(1, len(dataset.index)):
                for var in range(len(settings.pred_var)):
                    motor[var][ha][i] = settings.motor_model(ha, dataset.iloc[i], dataset.iloc[i-1])[var]

        # Drop irrelevant variables
        for column in dataset:
            if settings.vars_used.count(column) == 0:
                dataset.drop(column, axis=1, inplace=True)
            else:
                dataset[column] = pd.to_numeric(dataset[column])
        
        # Append motor controller outputs
        for var in range(len(settings.pred_var)):
            for ha in range(settings.numHA):
                dataset["controller-" + settings.pred_var[var] + "-" + str(ha)] = motor[var][ha]

        # Move predicted variables to the end for readability
        for var in range(len(settings.pred_var)):
            column_to_move = dataset.pop(settings.pred_var[var])
            dataset.insert(len(dataset.columns), settings.pred_var[var], column_to_move)
        
        # print("Relevant Variables:")
        # print(dataset)

        # Convert from series to supervised learning problem
        dataset = dataset.tail(-1) # Drop first row where motor model is undefined
        dataset = series_to_supervised(dataset, 1, 1)

        dataset_validation = pd.concat([dataset_validation, dataset], ignore_index=True)
        if iter < settings.training_set:
            training_size = len(dataset_validation.index)

        # print("Relevant Variables:")
        # print(dataset)
    
    print("Relevant data:")
    print(dataset_validation)

    # Normalize features
    scaler_validation = MinMaxScaler(feature_range=(0, 1))
    for col in dataset_validation.columns:
        # Normalize columns with the same metric using the same scaler
        is_la = False
        for var in range(len(settings.pred_var)):
            if col.startswith("controller-" + settings.pred_var[var] + "-") or col.startswith(settings.pred_var[var]):
                scaler_validation.fit(np.transpose([settings.pv_range[var]]))
                is_la = True
        
        if not is_la:
            scaler_validation.fit(np.transpose([dataset_validation[col]]))
        
        if not col.startswith("HA"):
            dataset_validation[col] = np.transpose(scaler_validation.transform(np.transpose([dataset_validation[col]])))[0]
        
    print("\n\nRelevant data: Scaled and reframed as a supervised learning problem\n\n")
    print(dataset_validation)

    # Pass in all data, to be split into validation & training sets
    makePredictions(dataset_validation, training_size)




def makePredictions(full_set, training_size):
    df_validation = full_set.loc[:, ~full_set.columns.str.startswith('HA')]

    assert df_validation.to_numpy().max() <= 1.1 and df_validation.to_numpy().min() >= -0.1

    # Split into X (inputs) and Y (outputs)
    validation_X = DataFrame()
    validation_Y = DataFrame()

    # Outputs: predicted variables
    for var in range(len(settings.pred_var)):
        validation_Y[settings.pred_var[var] + "(t)"] = df_validation[settings.pred_var[var] + "(t)"]
        
    # Inputs: everything else
    for col in df_validation:
        if not col in validation_Y.columns:
            validation_X[col] = df_validation[col]

    print(validation_X)
    print(validation_Y)
    
    # Take subset to use for training data
    df_train_X = validation_X.loc[0:training_size-1]
    df_train_Y = validation_Y.loc[0:training_size-1]

    # Split training set further into train and test sets and isolate values
    n_train_hours = math.floor(len(df_train_X.index) * 0.3)
    df_train_X, df_train_Y = df_train_X.values, df_train_Y.values
    train_X, train_Y = df_train_X[n_train_hours:], df_train_Y[n_train_hours:]
    test_X, test_Y = df_train_X[:n_train_hours], df_train_Y[:n_train_hours]

    X_validation = validation_X.values
    Y_validation = validation_Y.values

    print("Training set size")
    print(len(train_X))
    # print(train_X)
    # print(train_Y)
    print("Test set size")
    print(len(test_X))
    # print(test_X)
    # print(test_Y)
    print("Validation set size")
    print(len(X_validation))
    # print(X_validation)
    # print(Y_validation)

    # Reshape input to be 3D [samples, timesteps, features]
    df_train_X = df_train_X.reshape((df_train_X.shape[0], 1, df_train_X.shape[1]))
    train_X = train_X.reshape((train_X.shape[0], 1, train_X.shape[1]))
    test_X = test_X.reshape((test_X.shape[0], 1, test_X.shape[1]))
    X_validation = X_validation.reshape((X_validation.shape[0], 1, X_validation.shape[1]))

    # Early stopping
    es = EarlyStopping(monitor='val_loss', verbose=1, patience=settings.patience)

    numVar = len(settings.pred_var)
    
    # Design network
    model = Sequential()
    model.add(LSTM(128, input_shape=(train_X.shape[1], train_X.shape[2])))
    model.add(Dense(64, activation=keras.activations.sigmoid))
    model.add(Dense(64, activation=keras.activations.sigmoid))
    model.add(Dense(numVar, activation=keras.activations.sigmoid))
    model.compile(loss="mae", optimizer='adam')
    print(model.summary())

    # Fit network
    history = model.fit(train_X, train_Y, epochs=settings.train_time, batch_size=128, validation_data=(test_X, test_Y), verbose=0, shuffle=False, callbacks=[es])  # validation_split= 0.2)
    model.save(settings.folder + "model")
    
    # Plot history
    pyplot.plot(history.history['loss'], label='train_loss')
    pyplot.plot(history.history['val_loss'], label='validation_loss')
    pyplot.legend()
    pyplot.savefig("plots/loss.png")
    pyplot.clf()
    pyplot.close('all')

    # Make a prediction and plot results
    yhat_test = model.predict(df_train_X)
    yhat_valid = model.predict(X_validation)

    print("Predicted:")
    print(yhat_valid)

    print("Actual:")
    print(Y_validation)

    #### Generate expected trajectories using softmax weights ####
    test_la, valid_la = [], []
    for var in range(len(settings.pred_var)):
        test_la.append(yhat_test[:, var])
        valid_la.append(yhat_valid[:, var])

    #### METRICS: WEIGHTED BY SOFTMAX ####
    print("######## Metrics: Weighted by softmax ########")

    # Metrics for testing set
    log_obs = util.cum_log_obs(test_la, df_train_Y)
    print("Testing set average log obs: " + str(log_obs))

    # Metrics for validation set
    log_obs_valid = util.cum_log_obs(valid_la, Y_validation)
    log_obs_valid = (log_obs_valid * settings.validation_set - log_obs * settings.training_set) / (settings.validation_set - settings.training_set)
    print("Validation set average log obs: " + str(log_obs_valid))

    print("", flush=True)
    plotter.plotLA(valid_la, Y_validation)

# Run neural network
loadDataFrame()