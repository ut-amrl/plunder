# Standard required imports
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt

# scipy imports
from scipy.optimize import minimize

# objective function
# def func(x):
#     return (x-3) * (x+2)

# # initial guess
# x0 = np.array([1.5])
# # call optimization algorithm
# res = minimize(func, x0, method='nelder-mead', options={'xatol': 1e-8, 'disp': True}) # xatol: tolerance, disp: display messages to console

# print(res.x)


# an example with local minima (thus the initial guess matters) and multiple parameters
# def local_min(x):
#     return (x[0]+1) * (x[0]-1) * (x[0]+1) * (x[0]-1) * (x[1]+1) * (x[1]-1) * (x[1]+1) * (x[1]-1)

# x0 = np.array([0.5, 0.5])
# res = minimize(local_min, x0, method='nelder-mead', options={'xatol': 1e-8, 'disp': True})

# print(res.x)

# x0 = np.array([-0.5, -0.5])
# res = minimize(local_min, x0, method='nelder-mead', options={'xatol': 1e-8, 'disp': True})

# print(res.x)



# Gradient descent
# def gradient_descent(x):
#     obj = (x-3) * (x+2)
#     der = 2*x-1
#     return obj, der # objective function, derivative (gradient)

# x0 = np.array([1.5])
# # BFGS - gradient descent
# res = minimize(gradient_descent, x0, method='BFGS', jac=True, options={'disp': True})

# print(res.x)




# Gradient descent, estimated gradient
# def gradient_descent_est(x):
#     obj = (x-3) * (x+2)
#     return obj

# x0 = np.array([1.5])
# res = minimize(gradient_descent_est, x0, method='BFGS', options={'disp': True})

# print(res.x)





# Gradient descent with local minima and multiple parameters
def gradient_descent_hard(x):
    obj = (x[0]+1) * (x[0]-1) * (x[0]+1) * (x[0]-1) * (x[1]+1) * (x[1]-1) * (x[1]+1) * (x[1]-1)
    return obj

x0 = np.array([0.5, 0.5])
res = minimize(gradient_descent_hard, x0, method='BFGS', options={'disp': True})

print(res.x)

x0 = np.array([-0.5, -0.5])
res = minimize(gradient_descent_hard, x0, method='BFGS', options={'disp': True})

print(res.x)