import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from scipy import optimize
from scipy import special as sp
from multiprocessing import Pool

import math
import warnings
import time
import json

# TODO
# how to weight all examples equally, so as not to bias towards the ones with high / extreme values
# or just control the range of examples during data collection
# optimization methods are highly dependent on initial guess
# for most applications starting at 0 as the center of the log function is probably best?
# or is it cheating because that's the only constant in this specific simulation 
# calculate ideal log-likelihood for each ASP transition
# bounds for basin hopping and BFGS is broken (maybe my fault)
# for some reason ACC->CON works when I do it manually (with local BFGS starting at x_0 = 0) but not when PIPS
# maybe has to do with choosing the samples? that is the only difference
# in this manual test i take all the transitions compared to PIPS only taking from the window
# maybe just expand window idk

# ------- Parameters -----------------------------
opt_method = 0          # See below
enumerateSigns = True   # Equivalent to enumerating over > and <
print_debug = True      # Extra debugging info
initial_values = 4      # Initial values for x_0: 0 = all zeros, 1 = average, >1 = enumerate over random initial guesses (use this to specify how many)
num_cores = 16           # Number of processes to run in parallel
max_spread = 5.0        # Maximum absolute value of alpha (slope)
bounds_extension = 0.1  # Amount to search above and below extrema
print_warnings = False  # Debugging info
print_padding = 30      # Print customization

# optimization method
# 0: local
# 1: basin hopping
# 2: dual annealing
# 3: DIRECT

# -------- objective function --------------------
def log_loss(x):
    alpha = x[: len(x)//2]  # values of alpha (slope) for each conditional structure
    x_0 = x[len(x)//2 :]    # center of logistic function for each conditional structure

    log_loss = 0
    for i in range(len(y_j)):
        log_likelihood = - sp.logsumexp([0, - alpha[0] * (E_k[0][i] - x_0[0])])

        for j in range(len(clauses)):
            # Calculate likelihood for this clause
            log_likelihood_j = - sp.logsumexp([0, - alpha[j+1] * (E_k[j+1][i] - x_0[j+1])])

            if clauses[j] == 0: # AND: multiply likelihoods
                log_likelihood += log_likelihood_j 
            if clauses[j] == 1: # OR: add likelihoods
                log_likelihood = sp.logsumexp([log_likelihood, log_likelihood_j, log_likelihood+log_likelihood_j], b=[1, 1, -1])
        
        # Compute total log loss
        if y_j[i]:  # Satisfied transition
            log_loss -= log_likelihood
        else:       # Unsatisfied transition
            log_loss -= sp.logsumexp([0, log_likelihood], b=[1, -1])

    return log_loss


# ------- helper functions ----------------------
def extension(l, r): # list range
    return (max(l, r) - min(l, r)) * bounds_extension

def debug(str):
    if print_debug:
        print(str)

def print_with_padding(label, value):
    debug((label+" ").ljust(print_padding, "-")+" "+str(value))

# ---------- Callback ------------------------
def print_fun(x, f, accepted):
    # debug("at minimum %.4f accepted %d" % (f, int(accepted)))
    # debug("with parameters: ")
    # debug(x)
    return

# ---------- Bounds ------------------------
class Bounds:
    def __init__(self, xmin, xmax):
        self.xmax = np.array(xmax)
        self.xmin = np.array(xmin)
    def __call__(self, **kwargs):
        x = kwargs["x_new"]
        tmax = bool(np.all(x <= self.xmax))
        tmin = bool(np.all(x >= self.xmin))
        return tmax and tmin

def find_min_max(expression):
    
    lo = min(expression)
    lo_ind = np.argmin(expression)
    hi = max(expression)
    hi_ind = np.argmax(expression)

    lo_diff = hi
    hi_diff = lo
    for i in range(len(expression)):
        if not (y_j[i] == y_j[lo_ind]):
            lo_diff = min(lo_diff, expression[i])
        if not (y_j[i] == y_j[hi_ind]):
            hi_diff = max(hi_diff, expression[i])
    
    return (lo_diff - extension(lo_diff, hi_diff), hi_diff + extension(lo_diff, hi_diff))

# --------- Stepping (basin hopping) -------------------------
class TakeStep:
    def __init__(self, stepsize=0.5):
        self.stepsize = stepsize
        self.rng = np.random.default_rng()
    def __call__(self, x):
        mid = len(x) // 2
        s = self.stepsize
        x[:mid] += self.rng.uniform(-s, s, x[:mid].shape)
        x[mid:] += self.rng.uniform(-100*s, 100*s, x[mid:].shape)
        # x += (np.random.randint(2, size=x.shape)-0.5)*2
        return x

# ---------- Optimizer ------------------------

# Runs the optimizer, given some initial parameters
def run_optimizer_from_initial(init):
    if(opt_method == 0):        # Gradient descent - local optimization
        res = optimize.minimize(log_loss, init,
                                method='BFGS', options={'disp': print_debug, 'maxiter': 100})
    elif(opt_method == 1):      # Basin hopping - global optimization
        res = optimize.basinhopping(log_loss, init,
                                niter=100, T=100.0,
                                minimizer_kwargs=minimizer_kwargs, accept_test=bounds_obj, 
                                take_step=step_obj, callback=print_fun)
    elif(opt_method == 2):      # Dual annealing - global optimization
        res = optimize.dual_annealing(log_loss, bounds, x0=init, 
                                        maxiter=50, initial_temp=50000, 
                                        visit=3.0, accept=-5, 
                                        minimizer_kwargs=minimizer_kwargs)
    elif(opt_method == 3):      # DIRECT - global optimization
        res = optimize.direct(log_loss, bounds_arr, maxiter=10000)
    else:
        sys.exit("Please use a valid optimization method")

    print_with_padding("Optimal parameters", "|")
    debug(res.x)
    print_with_padding("Num iterations", res.nfev)
    print_with_padding("Minimum value", res.fun)
    debug("")

    return res

# Handles initialization and enumeration, then calls the optimizer
def run_optimizer(E_k_loc, y_j_loc, clauses_loc):
    global E_k, y_j, clauses, minimizer_kwargs, step_obj, bounds, bounds_arr, bounds_obj
    E_k = E_k_loc
    y_j = y_j_loc
    clauses = clauses_loc

    if(not print_warnings):
        warnings.filterwarnings('ignore')

    minimizer_kwargs = {"method": "BFGS"}

    step_obj = TakeStep()

    # ---------- Bounds --------------------

    # Bounds on alpha : defined by max_spread
    alpha_bounds = [(-max_spread, max_spread) for expression in E_k]

    # Bounds on x_0 : calculated from minimum/maximum and provided bounds extension
    x_0_bounds = [find_min_max(expression) for expression in E_k]

    # Setup bounds
    bounds = np.concatenate((alpha_bounds, x_0_bounds))
    bounds_lower = [b[0] for b in bounds]
    bounds_upper = [b[1] for b in bounds]
    print_with_padding("Bounds", "|")
    debug(bounds)
    bounds_obj = Bounds(bounds_lower, bounds_upper)
    bounds_arr = optimize.Bounds(bounds_lower, bounds_upper)

    # ---------- Optimizer ------------------------
    input = []

    for _ in range(max(1, initial_values)):

        # Initialization of x_0
        x_0_init = np.zeros(len(E_k)) # Initialize to 0s
        if initial_values == 1:       # Initialize to the average
            x_0_init = [sum(expression)/len(expression) for expression in E_k]
        elif initial_values > 1:      # Initialize randomly 
            x_0_init = [np.random.uniform(bound[0], bound[1]) for bound in x_0_bounds]

        for signs in range(pow(2, len(E_k))): # iterate over possible signs for alpha
            
            # Initialization of alpha
            alpha_init = []
            for i in range(len(E_k)):
                alpha_init.append(1 if (signs & (1 << i)) else -1)

            if not enumerateSigns: # Initialize to 0 (don't iterate over signs)
                alpha_init = np.zeros(len(E_k))
            
            init = np.concatenate((alpha_init, x_0_init))

            # Calling the optimizer
            input.append(init)
            
            if not enumerateSigns:
                break
        
        if initial_values <= 1:
            break
    
    print_with_padding("Initial values", "|")
    debug(input)
    
    # Run optimizer in parallel
    with Pool(num_cores) as p:
        output = p.map(run_optimizer_from_initial, input)
    
    bestRes = min(output, key=lambda i: i.fun)
    
    print_with_padding("Final parameters", "|")
    debug(bestRes.x)
    print_with_padding("Minimum value", bestRes.fun)

    return (bestRes.fun, list(bestRes.x))