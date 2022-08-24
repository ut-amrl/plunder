import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import math

from scipy import optimize

# ------- Parameters -----------------------------
max_spread = 10
max_threshold = 1000

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


# --------- optimizer -----------------------------
def run_optimizer():
    # -------- Initialization -------------------
    init = np.zeros(2 * len(E_k)) # Set initial center and "spread" to 0

    # -------- Bounds ---------------------------
    # Set center bounds
    bounds_lower = np.full(len(init), -1 * max_threshold)
    bounds_upper = np.full(len(init), max_threshold)
    for i in range(len(E_k)):
        # Set bounds on the spread -- keep some error
        bounds_lower[i] = -1 * max_spread
        bounds_upper[i] = max_spread

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
    
    # --------- Stepping -------------------------
    class TakeStep:
        def __init__(self, stepsize=0.5):
            self.stepsize = stepsize
            self.rng = np.random.default_rng()
        def __call__(self, x):
            mid = len(x) // 2
            s = self.stepsize
            x[:mid] += self.rng.uniform(-s, s, x[:mid].shape)
            x[mid:] += self.rng.uniform(-100*s, 100*s, x[mid:].shape)
            return x

    step_obj = TakeStep()

    # ---------- Callback ------------------------
    def print_fun(x, f, accepted):
        # print("at minimum %.4f accepted %d" % (f, int(accepted)))
        # print("with parameters: ")
        # print(x)
        return

    # -------- Optimization ----------------------
    minimizer_kwargs = {"method": "BFGS"}
    rng = np.random.default_rng()

    # local optimization
    # res = optimize.minimize(log_loss, init, method='BFGS', options={'disp': True})

    # global optimization: basin hopping
    # res = optimize.basinhopping(log_loss, init, niter=100, T=50.0, minimizer_kwargs=minimizer_kwargs, accept_test=bounds_obj, take_step=step_obj, callback=print_fun, seed=rng)

    # global optimization: dual annealing
    res = optimize.dual_annealing(log_loss, bounds_arr, x0=init, maxiter=2000, initial_temp=40000, visit=3.0, accept=-5, minimizer_kwargs=minimizer_kwargs)

    # global optimization: DIRECT algorithm - doesn't really work
    # res = optimize.direct(log_loss, bounds_arr, maxiter=1000)

    print("Optimal parameters: ")
    print(res.x)
    print("Minimum value: ")
    print(res.fun)
    print("Num iterations: ")
    print(res.nfev)
    return res

# -------- Tests, Problems, and Data -----------------------------

### Goal: synthesize x < 30 ----------> SUCCESS
# https://www.desmos.com/calculator/rymyh1m1gv

clauses = []
y_j = [True, False]
E_k = [
    [29, 31]
]

### Tests
assert abs(log_loss([2, 25]) - 12) < 0.001
assert abs(log_loss([4, 35]) - 24) < 0.001

# res = run_optimizer()
# assert abs(res.x[1] - 30) < 0.05



### Goal: synthesize (x > 10 && x < 20) ----------> SUCCESS

clauses = [ '&' ] # (p_1 & p_2)
y_j = [False, False, False, True, True, True, True, False, False, False, False] # whether or not each example satisfied the transition
E_k = [ 
    [-5, 0, 9, 11, 14, 15, 19, 21, 30, 40, 49],
    [-5, 0, 9, 11, 14, 15, 19, 21, 30, 40, 49],
] # value of E_k(s) for each example, for each predicate

### Tests
res = run_optimizer()



### Goal: synthesize (x > 10 && x < 20) || x > 50 ----------> FAIL
# https://www.desmos.com/calculator/1m79wd5h0e 

clauses = [ '&' , '|' ] # (p_1 & p_2) | p_3
y_j = [False, False, False, True, True, True, True, False, False, False, False, True, True, True] # whether or not each example satisfied the transition
E_k = [ 
    [-5, 0, 9, 11, 14, 15, 19, 21, 30, 40, 49, 51, 55, 70],
    [-5, 0, 9, 11, 14, 15, 19, 21, 30, 40, 49, 51, 55, 70],
    [-5, 0, 9, 11, 14, 15, 19, 21, 30, 40, 49, 51, 55, 70],
] # value of E_k(s) for each example, for each predicate

### Tests
assert abs(log_loss([2, -2, 1, 12, 18, 50]) - 4.9155) < 0.001
assert abs(log_loss([-1, -5, -1, 3, 18, 4]) - 176.66) < 0.001

# res = run_optimizer()