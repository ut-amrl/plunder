import gym
import numpy as np
import random

env = gym.make('highway-v0')
# env.reset(seed=474) # Setting the seed

# Probabilistic functions
def logistic(offset, slope, x):
    return 1.0/(1.0+np.exp(-slope*(x-offset)))

def sample(p):
    return random.random()<p

def asp_0(ego, closest, ha):
    arr = [env.action_type.actions_indexes["FASTER"], env.action_type.actions_indexes["SLOWER"], env.action_type.actions_indexes["LANE_LEFT"], env.action_type.actions_indexes["LANE_RIGHT"]]
    if sample(0.1):
        return arr[random.randint(0, 3)]
    return ha

def asp_1(ego, closest, ha):
    x, l_x, f_x, r_x = ego[1], closest[0][1], closest[1][1], closest[2][1] 
    vx = ego[3]

    # Synthesized ASP
    if ha == env.action_type.actions_indexes["FASTER"]:
        if sample(logistic(-32.676, -0.2111, x - r_x)):
            return env.action_type.actions_indexes["LANE_RIGHT"]
        if sample(logistic(28.274668, -0.179733, f_x - x)):
            return env.action_type.actions_indexes["SLOWER"]
    elif ha == env.action_type.actions_indexes["LANE_LEFT"]:
        if sample(logistic(34.280392, -0.161752, vx)):
            return env.action_type.actions_indexes["FASTER"]
        if sample(logistic(8.336246, -0.486861, x / closest[2][3])):
            return env.action_type.actions_indexes["LANE_RIGHT"]
        if sample(logistic(326.749756, -0.054127, r_x)):
            return env.action_type.actions_indexes["SLOWER"]
    elif ha == env.action_type.actions_indexes["LANE_RIGHT"]:
        if sample(logistic(19.262970, -0.212244, r_x - x)):
            return env.action_type.actions_indexes["FASTER"]
        if sample(logistic(749.367737, 0.012648, l_x)):
            return env.action_type.actions_indexes["LANE_LEFT"]
        if sample(logistic(582.408081, 0.026336, l_x)):
            return env.action_type.actions_indexes["SLOWER"]
    elif ha == env.action_type.actions_indexes["SLOWER"]:
        if sample(logistic(57.475712, 0.160559, f_x - x)):
            return env.action_type.actions_indexes["FASTER"]
        if sample(logistic(-54.074814, -0.294189, closest[0][3])):
            return env.action_type.actions_indexes["LANE_LEFT"]
        if sample(logistic(-33.813507, -0.748582, x - r_x)):
            return env.action_type.actions_indexes["LANE_RIGHT"]

    return ha

def asp_3(ego, closest, ha):
    x, l_x, f_x, r_x = ego[1], closest[0][1], closest[1][1], closest[2][1] 
    vx = ego[3]
    l_vx, f_vx, r_vx = closest[0][3], closest[1][3], closest[2][3]

    # Synthesized ASP
    if ha == env.action_type.actions_indexes["FASTER"]:
        if sample(logistic(29.406006, 3.242685, r_x - x)):
            return env.action_type.actions_indexes["LANE_RIGHT"]
        if sample(logistic(0.511086, -59.398228, (f_x - x) / (2 * vx))):
            return env.action_type.actions_indexes["SLOWER"]
    elif ha == env.action_type.actions_indexes["LANE_RIGHT"]:
        if sample(logistic(-118.355446, 14.204762, x - f_x)) and sample(logistic(27.424053, -13.509148, r_x - x)):
            return env.action_type.actions_indexes["FASTER"]
    elif ha == env.action_type.actions_indexes["SLOWER"]:
        if sample(logistic(51.459400, 0.270894, f_x - x)):
            return env.action_type.actions_indexes["FASTER"]
        if sample(logistic(-35.224407, -0.708823, x - r_x)):
            return env.action_type.actions_indexes["LANE_RIGHT"]

    return ha

def asp_8(ego, closest, ha):
    x, l_x, f_x, r_x = ego[1], closest[0][1], closest[1][1], closest[2][1] 
    vx = ego[3]
    l_vx, f_vx, r_vx = closest[0][3], closest[1][3], closest[2][3]

    # Synthesized ASP
    if ha == env.action_type.actions_indexes["FASTER"]:
        if sample(logistic(29.516665, 3.254827, r_x - x)):
            return env.action_type.actions_indexes["LANE_RIGHT"]
        if sample(logistic(1.027847, -93.105392, (f_x - x) / vx)):
            return env.action_type.actions_indexes["SLOWER"]
    elif ha == env.action_type.actions_indexes["LANE_RIGHT"]:
        if sample(logistic(118.379364, -13.955011, x - f_x)) and sample(logistic(27.389425, -13.41971, r_x - x)):
            return env.action_type.actions_indexes["FASTER"]
    elif ha == env.action_type.actions_indexes["SLOWER"]:
        if sample(logistic(-2.341030, -68.667282, (x - f_x) / f_vx)):
            return env.action_type.actions_indexes["FASTER"]
        if sample(logistic(-34.740185, -0.702651, x - r_x)):
            return env.action_type.actions_indexes["LANE_RIGHT"]

    return ha
