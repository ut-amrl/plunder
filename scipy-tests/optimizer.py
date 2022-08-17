import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import math

from scipy import optimize

max_spread = 10
max_threshold = 1000

# Goal: synthesize (x > 10 && x < 20) || x > 50

clauses = [ '&' , '|' ] # (p_1 & p_2) | p_3
y_j = [False, False, False, True, True, True, True, False, False, False, False, True, True, True] # whether or not each example satisfied the transition
E_k = [ 
    [-5, 0, 9, 11, 14, 15, 19, 21, 30, 40, 49, 51, 55, 70],
    [-5, 0, 9, 11, 14, 15, 19, 21, 30, 40, 49, 51, 55, 70],
    [-5, 0, 9, 11, 14, 15, 19, 21, 30, 40, 49, 51, 55, 70],
] # value of E_k(s) for each example


# Goal: synthesize x < 30

# clauses = []
# y_j = [True, False]
# E_k = [
#     [29, 31]
# ]


# objective function
def log_loss(x):
    alpha = x[: len(x)//2] # values of alpha for each conditional structure
    x_0 = x[len(x)//2 :] # center of logistic function for each conditional structure

    log_loss = 0
    for i in range(len(y_j)):
        
        likelihood = 1.0 / (1.0 + pow(math.e, - alpha[0] * (E_k[0][i] - x_0[0])))
        for j in range(len(clauses)):
            likelihood_j = 1.0 / (1.0 + pow(math.e, - alpha[j+1] * (E_k[0][i] - x_0[j+1])))
            if(clauses[j] == '&'): # AND
                likelihood = likelihood * likelihood_j
            if(clauses[j] == '|'): # OR
                likelihood = likelihood + likelihood_j - likelihood * likelihood_j

        if y_j[i]:
            log_loss -= math.log(max(likelihood, 1e-308)) # cap at small value to prevent undefined behavior
        else:
            log_loss -= math.log(max(1 - likelihood, 1e-308))

    return log_loss

init = np.zeros(2 * len(E_k))
bounds_lower = np.full(len(init), -1000)
bounds_upper = np.full(len(init), 1000)
for i in range(len(E_k)):
    init[i] = 0.05 # Ensure sufficient spread during initialization
    bounds_lower[i] = -10 # Set bounds on the spread -- keep some error
    bounds_upper[i] = 10


class MyBounds:
    def __init__(self):
        return
    def __call__(self, **kwargs):
        x = kwargs["x_new"]
        for i in range(len(x) // 2):
            if x[i] < -max_spread or x[i] > max_spread:
                return False
            if x[i + len(x) // 2] < -max_threshold or x[i + len(x) // 2] > max_threshold:
                return False
        
        return True

minimizer_kwargs = {"method": "BFGS"}
mybounds = MyBounds()
bounds = optimize.Bounds(bounds_lower, bounds_upper)
# res = optimize.minimize(log_loss, init, method='BFGS', options={'disp': True})
# res = optimize.basinhopping(log_loss, init, T=50.0, stepsize=5.0, minimizer_kwargs=minimizer_kwargs, accept_test=mybounds)
res = optimize.dual_annealing(log_loss, bounds, maxiter=1000, initial_temp=50000, visit=3.0, minimizer_kwargs=minimizer_kwargs)

print(res.x)