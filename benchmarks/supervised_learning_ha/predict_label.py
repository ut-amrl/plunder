# https://machinelearningmastery.com/multivariate-time-series-forecasting-lstms-keras/
# Credit to Justina Lam for providing a baseline for a good chunk of this code
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
from sklearn.metrics import mean_squared_error

import settings


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

        out = open("formatted_data.csv", "w")
        for line in Lines:
            out.write(line + "\n")

        out.close()
        f.close() 

        # Load dataset
        dataset = pd.read_csv("formatted_data.csv")

        # print("Original dataset:")
        # print(dataset)

        # Compute outputs from discrete motor controllers
        motor = np.empty((settings.numHA, len(dataset.index)))
        for ha in range(settings.numHA):
            for i in range(1, len(dataset.index)):
                motor[ha][i] = settings.motor_model(ha, dataset.iloc[i], dataset.iloc[i-1])[0]

        # Drop irrelevant variables
        for column in dataset:
            if settings.vars_used.count(column) == 0:
                dataset.drop(column, axis=1, inplace=True)
            else:
                dataset[column] = pd.to_numeric(dataset[column])
        
        # Append motor controller outputs
        for ha in range(settings.numHA):
            dataset["controller-" + str(ha)] = motor[ha]

        # Move predicted variables to the end for readability
        if not settings.pred_var1 == None:
            column_to_move = dataset.pop(settings.pred_var1)
            dataset.insert(len(dataset.columns), settings.pred_var1, column_to_move)
        if not settings.pred_var2 == None:
            column_to_move = dataset.pop(settings.pred_var2)
            dataset.insert(len(dataset.columns), settings.pred_var2, column_to_move)
        
        # print("Relevant Variables:")
        # print(dataset)

        # Convert from series to supervised learning problem
        dataset = dataset.tail(-1) # Drop first row where motor model is undefined
        dataset = series_to_supervised(dataset, 4, 1)

        dataset_validation = pd.concat([dataset_validation, dataset], ignore_index=True)
        if iter < settings.training_set:
            training_size = len(dataset_validation.index)
    
    print("Relevant data:")
    print(dataset_validation)

    # Normalize features
    scaler_validation = MinMaxScaler(feature_range=(0, 1))
    for col in dataset_validation.columns:
        # Normalize columns with the same metric using the same scaler
        if col.startswith("controller-") or col.startswith(settings.pred_var1):
            scaler_validation.fit(np.transpose([settings.pv1_range]))
        else: # Perform regular fit_transform
            scaler_validation.fit(np.transpose([dataset_validation[col]]))
        dataset_validation[col] = np.transpose(scaler_validation.transform(np.transpose([dataset_validation[col]])))[0]
        
    print("\n\nRelevant data: Scaled and reframed as a supervised learning problem\n\n")
    print(dataset_validation)

    assert dataset_validation.to_numpy().max() <= 1 and dataset_validation.to_numpy().min() >= 0

    # Pass in all data, to be split into validation & training sets
    makePredictions(dataset_validation, training_size)

fig_test = Figure()
fig_valid = Figure()

figs = []  # fig_test, fig_valid
results = []  # datetime, yhat_valid, y_validation
toReturn = []

def makePredictions(df_validation, training_size):
    scaler = MinMaxScaler(feature_range=(0, 1))

    min_la1 = df_validation[settings.pred_var1+'(t)'].min()
    mean_la1 = df_validation[settings.pred_var1+'(t)'].mean()
    max_la1 = df_validation[settings.pred_var1+'(t)'].max()

    df_la1_levels = pd.DataFrame({settings.pred_var1: [min_la1, mean_la1, max_la1]})

    la1_scaled = scaler.fit_transform(df_la1_levels.values)
    la1_scaled = la1_scaled[1][0]

    la1_levels_scaled = [(1.5 * la1_scaled), (la1_scaled), (0.5 * la1_scaled), (0.25 * la1_scaled)]
    la1_colors = ['r', 'tab:orange', 'y', 'g']
    la1_labels = ['150% Mean', 'Mean LA 1', '50% Mean', '25% Mean']

    # Split into X (inputs) and Y (outputs)
    validation_X = DataFrame()
    validation_Y = DataFrame()

    # Outputs: predicted variables and discrete motor controller outputs (for use in the loss function)
    if not settings.pred_var1 == None:
        validation_Y[settings.pred_var1 + "(t)"] = df_validation[settings.pred_var1 + "(t)"]
    if not settings.pred_var2 == None:
        validation_Y[settings.pred_var2 + "(t)"] = df_validation[settings.pred_var2 + "(t)"]
    for ha in range(settings.numHA):
        validation_Y["controller-" + str(ha) + "(t)"] = df_validation["controller-" + str(ha) + "(t)"]
    
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
    n_train_hours = math.floor(len(df_train_X.index) * 0.1)
    train_X, train_Y = df_train_X.values[n_train_hours:], df_train_Y.values[n_train_hours:]
    test_X, test_Y = df_train_X.values[:n_train_hours], df_train_Y.values[:n_train_hours]

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
    train_X = train_X.reshape((train_X.shape[0], 1, train_X.shape[1]))
    test_X = test_X.reshape((test_X.shape[0], 1, test_X.shape[1]))
    X_validation = X_validation.reshape((X_validation.shape[0], 1, X_validation.shape[1]))

    # Custom metric
    mse = tensorflow.keras.losses.MeanSquaredError()
    class CustomAccuracy(keras.losses.Loss):
        def __init__(self):
            super().__init__()
        def call(self, y_true, y_pred):
            # offset = tensorflow.math.add(y_pred, 1E-8)

            # Perform a weighted sum of all controllers
            sum = tensorflow.reduce_sum(tensorflow.multiply(y_pred, y_true[:, 1:]), 1)
            
            # Calculate mean squared error from observed LA
            return mse(sum, y_true[:, :1])

    # Design network
    
    model = Sequential()
    model.add(LSTM(128, input_shape=(train_X.shape[1], train_X.shape[2])))
    model.add(Dense(64, activation=keras.activations.sigmoid))
    model.add(Dense(16, activation=keras.activations.sigmoid))
    model.add(Dense(settings.numHA, activation=keras.activations.softmax)) # Output: one-hot encoding of each HA, softmax to convert to a vector of weights
    model.compile(loss=CustomAccuracy(), optimizer='adam')

    print(model.summary())
    # Fit network
    history = model.fit(train_X, train_Y, epochs=settings.train_time, batch_size=128, validation_data=(test_X, test_Y), verbose=2, shuffle=False)  # validation_split= 0.2)

    # Plot history
    pyplot.plot(history.history['loss'], label='train_loss')
    pyplot.plot(history.history['val_loss'], label='validation_loss')
    pyplot.legend()
    pyplot.savefig("loss.png")
    pyplot.show()

    # Make a prediction and plot results
    yhat_test = model.predict(test_X)
    # print("TEST_X")
    # print(test_X.shape)
    # print("YHAT_TEST")
    # print(yhat_test)

    pyplot.plot(test_Y, label='test_y')
    pyplot.plot(yhat_test, label='yhat_test')
    for laInd in range(len(la1_levels_scaled)):
        pyplot.axhline(y=la1_levels_scaled[laInd], color=la1_colors[laInd],
                       linestyle='-', label=la1_labels[laInd])
    pyplot.legend()
    pyplot.savefig("test.png")
    pyplot.show()

    # Plot and evaluate prediction results
    axis_test = fig_test.add_subplot(1, 1, 1)
    axis_test.plot(test_Y, label='Actual', linewidth=2)
    axis_test.plot(yhat_test, label='Predicted', linewidth=2.5, alpha=0.6, color='tab:pink')
    # for laInd in range(len(la1_levels_scaled)):
    #     axis_test.axhline(y=la1_levels_scaled[laInd], color=la1_colors[laInd], linestyle='-', label=la1_labels[laInd])

    yhat_valid = model.predict(X_validation)

    pyplot.plot(Y_validation, label='y_validation')
    pyplot.plot(yhat_valid, label='yhat_valid')
    # for laInd in range(len(la1_levels_scaled)):
    #     pyplot.axhline(y=la1_levels_scaled[laInd], color=la1_colors[laInd], linestyle='-', label=la1_labels[laInd])
    pyplot.legend()
    pyplot.savefig("validation.png")
    pyplot.show()

    axis_valid = fig_valid.add_subplot(1, 1, 1)
    axis_valid.plot(Y_validation, label='Actual', linewidth=2)
    axis_valid.plot(yhat_valid, label='Predicted',
                    linewidth=3, alpha=0.7, color='tab:pink')
    # for laInd in range(len(la1_levels_scaled)):
    #     axis_valid.axhline(y=la1_levels_scaled[laInd], color=la1_colors[laInd], linestyle='-', label=la1_labels[laInd], linewidth=1)

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
            max_y_validation = max(max(Y_validation[i], Y_validation[i+1]), max(Y_validation[i+2], Y_validation[i+3]))

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
    inv_yhat = np.concatenate((yhat_valid, X_validation[:, 1:]), axis=1)

    scaler = MinMaxScaler(feature_range=(0, 1)).fit(inv_yhat)

    inv_yhat = scaler.inverse_transform(inv_yhat)
    inv_yhat = inv_yhat[:, 0]

    # Invert scaling for actual
    Y_validation = Y_validation.reshape((len(Y_validation), 1))
    inv_y = np.concatenate((Y_validation, X_validation[:, 1:]), axis=1)

    inv_y = scaler.inverse_transform(inv_y)
    inv_y = inv_y[:, 0]

    # Calculate RMSE
    rmse = math.sqrt(mean_squared_error(inv_y, inv_yhat))
    print('Test RMSE: %.3f' % rmse)

# Run neural network
loadDataFrame()