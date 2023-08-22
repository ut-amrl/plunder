import gym
import numpy as np
import random

env = gym.make('highway-v0')

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
        if sample(logistic(71.567871, 0.046884, l_x - f_x)):
            return env.action_type.actions_indexes["LANE_LEFT"]
        if sample(logistic(-16.056057, 0.107517, x - f_x)):
            return env.action_type.actions_indexes["LANE_RIGHT"]
        if sample(logistic(25.797348, -0.141676, f_x - x)):
            return env.action_type.actions_indexes["SLOWER"]
    elif ha == env.action_type.actions_indexes["LANE_LEFT"]:
        if sample(logistic(-65.718407, -0.048061, x - f_x)):
            return env.action_type.actions_indexes["FASTER"]
        if sample(logistic(140.377899, 0.021186, r_x - l_x)):
            return env.action_type.actions_indexes["LANE_RIGHT"]
        if sample(logistic(28.491735, 0.050957, r_x - l_x)):
            return env.action_type.actions_indexes["SLOWER"]
    elif ha == env.action_type.actions_indexes["LANE_RIGHT"]:
        if sample(logistic(-74.275177, -0.048392, x - f_x)):
            return env.action_type.actions_indexes["FASTER"]
        if sample(logistic(70.051857, 0.025387, l_x - r_x)):
            return env.action_type.actions_indexes["LANE_LEFT"]
        if sample(logistic(17.227783, -0.065374, r_x - x)):
            return env.action_type.actions_indexes["SLOWER"]
    elif ha == env.action_type.actions_indexes["SLOWER"]:
        if sample(logistic(-51.172897, -0.152553, x - f_x)):
            return env.action_type.actions_indexes["FASTER"]
        if sample(logistic(40.806450, 0.104829, l_x - f_x)):
            return env.action_type.actions_indexes["LANE_LEFT"]
        if sample(logistic(-58.123692, -0.082703, x - r_x)):
            return env.action_type.actions_indexes["LANE_RIGHT"]

    return ha

def asp_3(ego, closest, ha):
    x, l_x, f_x, r_x = ego[1], closest[0][1], closest[1][1], closest[2][1] 
    vx = ego[3]
    l_vx, f_vx, r_vx = closest[0][3], closest[1][3], closest[2][3]

    # Synthesized ASP
    if ha == env.action_type.actions_indexes["FASTER"]:
        if sample(logistic(-38.659206, 0.829370, x - f_x)) and sample(logistic(11.460945, 0.447564, l_x - f_x)):
            return env.action_type.actions_indexes["LANE_LEFT"]
        if sample(logistic(45.382179, 0.053571, r_x - l_x)) and sample(logistic(-38.121521, 0.335502, x - f_x)):
            return env.action_type.actions_indexes["LANE_RIGHT"]
        if sample(logistic(-34.034515, 0.285341, x - f_x)):
            return env.action_type.actions_indexes["SLOWER"]
    elif ha == env.action_type.actions_indexes["LANE_LEFT"]:
        if sample(logistic(-56.613659, -0.102819, x - f_x)):
            return env.action_type.actions_indexes["FASTER"]
        if sample(logistic(-118.475563, -0.058139, x - f_x)):
            return env.action_type.actions_indexes["LANE_RIGHT"]
        if sample(logistic(61.980194, -1.442670, f_x - x)) and sample(logistic(-3.111790, 0.186741, r_x - l_x)):
            return env.action_type.actions_indexes["SLOWER"]
    elif ha == env.action_type.actions_indexes["LANE_RIGHT"]:
        if sample(logistic(-62.395676, -0.079893, x - f_x)):
            return env.action_type.actions_indexes["FASTER"]
        if sample(logistic(105.946220, 0.025125, f_x - r_x)):
            return env.action_type.actions_indexes["LANE_LEFT"]
        if sample(logistic(1.082092, -9.567812, (r_x - x) / vx)):
            return env.action_type.actions_indexes["SLOWER"]
    elif ha == env.action_type.actions_indexes["SLOWER"]:
        if sample(logistic(49.258362, 0.135073, f_x - x)):
            return env.action_type.actions_indexes["FASTER"]
        if sample(logistic(-1.105389, -4.461890, (f_x - l_x) / vx)):
            return env.action_type.actions_indexes["LANE_LEFT"]
        if sample(logistic(15.713934, 0.293426, f_vx)) and sample(logistic(-31.543020, -0.289382, x - r_x)):
            return env.action_type.actions_indexes["LANE_RIGHT"]

    return ha

def asp_8(ego, closest, ha):
    x, l_x, f_x, r_x = ego[1], closest[0][1], closest[1][1], closest[2][1] 
    vx = ego[3]
    l_vx, f_vx, r_vx = closest[0][3], closest[1][3], closest[2][3]

    # Synthesized ASP
    if ha == env.action_type.actions_indexes["FASTER"]:
        if sample(logistic(-39.018066, 1.239720, x - f_x)) and sample(logistic(8.335925, 6.518748, l_x - f_x)):
            return env.action_type.actions_indexes["LANE_LEFT"]
        if sample(logistic(52.884239, 0.070358, r_x - l_x)) and sample(logistic(-39.628555, 0.397472, x - f_x)):
            return env.action_type.actions_indexes["LANE_RIGHT"]
        if sample(logistic(0.976069, -28.409134, (f_x - x) / vx)):
            return env.action_type.actions_indexes["SLOWER"]
    elif ha == env.action_type.actions_indexes["LANE_LEFT"]:
        if sample(logistic(59.708252, 0.189650, f_x - x)):
            return env.action_type.actions_indexes["FASTER"]
        if False:
            return env.action_type.actions_indexes["LANE_RIGHT"]
        if sample(logistic(60.649853, -8.671058, f_x - x)) and sample(logistic(-8.006697, 8.428448, r_x - l_x)):
            return env.action_type.actions_indexes["SLOWER"]
    elif ha == env.action_type.actions_indexes["LANE_RIGHT"]:
        if sample(logistic(-66.188805, -0.072145, x - f_x)):
            return env.action_type.actions_indexes["FASTER"]
        if sample(logistic(40.839100, 0.298081, f_x - l_x)):
            return env.action_type.actions_indexes["LANE_LEFT"]
        if sample(logistic(11.456509, -20.497993, r_x - l_x)) and sample(logistic(-1.026174, 10.862728, (x - r_x) / vx)):
            return env.action_type.actions_indexes["SLOWER"]
    elif ha == env.action_type.actions_indexes["SLOWER"]:
        if sample(logistic(-49.595589, -0.119454, x - f_x)):
            return env.action_type.actions_indexes["FASTER"]
        if sample(logistic(55.706520, -12.193183, vx + f_vx)) and sample(logistic(-0.436302, -36.663643, (f_x - l_x) / vx)):
            return env.action_type.actions_indexes["LANE_LEFT"]
        if sample(logistic(17.366369, 0.563735, f_vx)) and sample(logistic(-3.205035, -0.790818, f_x - r_x)):
            return env.action_type.actions_indexes["LANE_RIGHT"]

    return ha
