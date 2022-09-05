# OUTDATED: See pips for the latest version



import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import math

from scipy import optimize
import warnings
import time

# ------- Parameters -----------------------------
opt_method = 0
enumerateSigns = False
print_debug = False

max_spread = 10
bounds_extension = 0.1
print_warnings = False
print_padding = 30

# optimization method
# 0: local
# 1: basin hopping
# 2: dual annealing
# 3: DIRECT

# ------- Variable declarations -----------------------------
E_k = []
y_j = []
clauses = []

# -------- objective function --------------------
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

        # cap at some value to prevent rounding to 0/1 (can cause undefined behavior)
        likelihood = max(likelihood, 1e-16)
        likelihood = min(likelihood, 1 - (1e-16))

        if y_j[i]:
            log_loss -= math.log(likelihood) # cap at small value to prevent undefined behavior
        else:
            log_loss -= math.log(1 - likelihood)

    return log_loss



# ------- helper functions ----------------------
def lr(l): # list range
    return max(l)-min(l)

def debug(str):
    if print_debug:
        print(str)

def print_with_padding(label, value):
    debug((label+" ").ljust(print_padding, "-")+" "+str(value))

# --------- optimizer -----------------------------
def run_optimizer(E_k_loc, y_j_loc, clauses_loc):
    global E_k, y_j, clauses
    E_k = E_k_loc
    y_j = y_j_loc
    clauses = clauses_loc

    if(not print_warnings):
        warnings.filterwarnings('ignore')

    minimizer_kwargs = {"method": "BFGS"}
    rng = np.random.default_rng()

    # ---------- Bounds --------------------
    x_0_bounds = [(-max_spread, max_spread) for expression in E_k]
    e_bounds = [(min(expression)-lr(expression)*bounds_extension, max(expression)+lr(expression)*bounds_extension) for expression in E_k]
    bounds = np.concatenate((x_0_bounds, e_bounds))
    bounds_lower = [b[0] for b in bounds]
    bounds_upper = [b[1] for b in bounds]
    print_with_padding("Bounds", "|")
    debug(bounds)

    class Bounds:
        def __init__(self, xmax=bounds_upper, xmin=bounds_lower):
            self.xmax = np.array(xmax)
            self.xmin = np.array(xmin)
        def __call__(self, **kwargs):
            x = kwargs["x_new"]
            tmax = bool(np.all(x <= self.xmax))
            tmin = bool(np.all(x >= self.xmin))
            return tmax and tmin

    bounds_obj = Bounds()
    bounds_arr = optimize.Bounds(bounds_lower, bounds_upper)
    
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

    step_obj = TakeStep()

    # ---------- Callback ------------------------
    def print_fun(x, f, accepted):
        # debug("at minimum %.4f accepted %d" % (f, int(accepted)))
        # debug("with parameters: ")
        # debug(x)
        return

    bestRes = -1
    for signs in range(pow(2, len(E_k))): # iterate over possible signs (corresponds to < and >)
        
        # ---------- Initialization ------------
        x_0_init = []
        for i in range(len(E_k)):
            x_0_init.append(1 if (signs & (1 << i)) else -1)

        if not enumerateSigns:
            x_0_init = np.zeros(len(E_k))
        
        e_init = [sum(expression)/len(expression) for expression in E_k]
        init = np.concatenate((x_0_init, e_init))
        print_with_padding("Initial values", "|")
        debug(init)

        # -------- Optimization ----------------------

        if(opt_method == 0):
            res = optimize.minimize(log_loss, init, 
                                    method='BFGS', options={'disp': True})
        elif(opt_method == 1):
            res = optimize.basinhopping(log_loss, init,
                                    niter=200, T=40000.0,
                                    minimizer_kwargs=minimizer_kwargs, accept_test=bounds_obj, 
                                    take_step=step_obj, callback=print_fun, seed=rng)
        elif(opt_method == 2):
            res = optimize.dual_annealing(log_loss, bounds, x0=init, 
                                            maxiter=1000, initial_temp=40000, 
                                            visit=3.0, accept=-5, 
                                            minimizer_kwargs=minimizer_kwargs)
        elif(opt_method == 3):
            res = optimize.direct(log_loss, bounds_arr, 
                                    maxiter=10000)
        else:
            sys.exit("Please use a valid optimization method")

        print_with_padding("Optimal parameters", "|")
        debug(res.x)
        print_with_padding("Num iterations", res.nfev)
        print_with_padding("Minimum value", res.fun)
        debug("")

        if bestRes == -1 or res.fun < bestRes.fun:
            bestRes = res
        
        if not enumerateSigns:
            break
    
    print_with_padding("Final parameters", "|")
    debug(bestRes.x)
    print_with_padding("Minimum value", bestRes.fun)

    return (bestRes.fun, list(bestRes.x))