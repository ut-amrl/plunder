import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import math

from scipy import optimize
import warnings
import time

# --------------- TODO ------------------------
# other option: only solve for threshold, not for spread
# define clauses as a tree instead a list of all & and | being left associative
# run more benchmarks
# interface with C++


# ------- Parameters -----------------------------
max_spread = 50
bounds_extension = 0.1
opt_method = 3
print_warnings = False
print_padding = 30

# optimization method
# 0: local
# 1: basin hopping
# 2: dual annealing
# 3: DIRECT



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
# list range
def lr(l):
    return max(l)-min(l)

def print_with_padding(label, value):
    print((label+" ").ljust(print_padding, "-")+" "+str(value))



# --------- optimizer -----------------------------
def run_optimizer(E_k, y_j, clauses, signs):
    
    # ---------- Initialization ------------
    x_0_init = signs
    e_init = [sum(expression)/len(expression) for expression in E_k]
    init = np.concatenate((x_0_init, e_init))
    print_with_padding("Initial values", "|")
    print(init)

    # ---------- Bounds --------------------
    x_0_bounds = [(max_spread/2*(n-1), max_spread/2*(n+1)) for n in x_0_init]
    e_bounds = [(min(expression)-lr(expression)*bounds_extension, max(expression)+lr(expression)*bounds_extension) for expression in E_k]
    bounds = np.concatenate((x_0_bounds, e_bounds))
    bounds_lower = [b[0] for b in bounds]
    bounds_upper = [b[1] for b in bounds]
    print_with_padding("Bounds", "|")
    print(bounds)

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
        # print("at minimum %.4f accepted %d" % (f, int(accepted)))
        # print("with parameters: ")
        # print(x)
        return

    # -------- Optimization ----------------------
    minimizer_kwargs = {"method": "BFGS"}
    rng = np.random.default_rng()

    # local optimization - fastest but local so may not always work
    # global optimization: basin hopping - slowest
    # global optimization: dual annealing - good balance
    # global optimization: DIRECT algorithm - fast but bad lol

    if(opt_method == 0):
         res = optimize.minimize(log_loss, init, 
                                method='BFGS', options={'disp': True})
    elif(opt_method == 1):
         res = optimize.basinhopping(log_loss, init,
                                niter=1000, T=1000.0,
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
    print(res.x)
    # print_with_padding("Num iterations", res.nfev)
    print_with_padding("Minimum value", res.fun)
    return res

# -------- Tests, Problems, and Data -----------------------------

if(not print_warnings):
    warnings.filterwarnings('ignore')

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
# res = run_optimizer()








### Goal: synthesize (x > 10 && x < 20) || x > 50 ----------> SUCCESS (after tweaking)
# https://www.desmos.com/calculator/1m79wd5h0e
# https://www.desmos.com/calculator/ttucfvkvlp


# ---------- Define examples -------------------------------------
y_j = [False, False, False, True, True, True, True, False, False, False, False, True, True, True] # whether or not each example satisfied the transition
E_k = [ 
    [-5, 0, 9, 11, 14, 15, 19, 21, 30, 40, 49, 51, 55, 70],
    [-5, 0, 9, 11, 14, 15, 19, 21, 30, 40, 49, 51, 55, 70],
    [-5, 0, 9, 11, 14, 15, 19, 21, 30, 40, 49, 51, 55, 70],
] # value of E_k(s) for each example, for each predicate

# now introduce some error
# it works
E_k = [ 
    [-5, 0, 9, 11, 14, 35, 19, 21, 30, 40, 49, 51, 55, 70],
    [-5, 0, 9, 11, 14, 35, 19, 21, 30, 40, 49, 51, 55, 70],
    [-5, 0, 9, 11, 14, 35, 19, 21, 30, 40, 49, 51, 55, 70],
] # value of E_k(s) for each example, for each predicate

# ---------- Define equation ------------------------------------
# clauses have left associativity?
clauses = [ '&' , '|' ]     # (p_1 & p_2) | p_3
signs = [1, -1, 1]          # p_1 = e_1 > n_1
                            # p_2 = e_2 < n_2
                            # p_3 = e_3 > n_3

### ----------------- Tests ---------------------------------------
# assert abs(log_loss([2, -2, 1, 12, 18, 50]) - 4.9155) < 0.001
# assert abs(log_loss([-1, -5, -1, 3, 18, 4]) - 176.66) < 0.001
# assert abs(log_loss([-0.09, 0.3, 0.3, 30, 3, 53]) - 6.7191) < 0.001
# assert abs(log_loss([3, -3, 8, 10, 20, 50]) - .195) < 0.001
# assert abs(log_loss([ 9.17704836, -9.96464641,  9.30102843, 10.07471293, 20.03487693, 49.95211582]) - 0) < 0.001

start = time.perf_counter()
res = run_optimizer(E_k, y_j, clauses, signs)
end = time.perf_counter()

print_with_padding("Time Elapsed", end-start)

