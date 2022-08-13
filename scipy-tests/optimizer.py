import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import math

from scipy.optimize import minimize

y_j = [True, False] # whether or not each example satisfied the transition
E_k = [ [20, 30] ] # value of E_k(s) for each example

# objective function
def log_loss(x):
    log_loss = 0
    for i in range(len(y_j)):
        loss = 1.0 / (1.0 + pow(math.e, - x[0] * (E_k[0][i] - x[1])))
        print(loss)
        if y_j[i]:
            log_loss -= math.log(loss)
        else:
            log_loss -= math.log(1 - loss)
    return log_loss

guess = np.array([0.1, 5])
res = minimize(log_loss, guess, method='BFGS', options={'disp': True})

print(res.x)