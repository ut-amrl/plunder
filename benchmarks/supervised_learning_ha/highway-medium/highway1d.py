import gym
import highway_env
from matplotlib import pyplot as plt
import os
import numpy as np
import math
import random

env = gym.make('highway-v0')
env.config['lanes_count']=1
env.config['vehicles_count']=1
env.config['simulation_frequency']=100
env.config['policy_frequency']=10
env.config['observation']={
    'type': 'Kinematics',
    'vehicles_count': 2,
    'features': ['presence', 'x', 'vx'],
    'absolute': False
}

highway_env.highway_env.envs.ControlledVehicle.DELTA_SPEED = 100
env.reset()

# observations
# ego vehicle:      presence, x, vx
# other vehicle:    presence, x, vx        (relative to ego)


# Probabilistic functions
def logistic(slope, offset, x):
    return 1.0/(1.0+np.exp(-slope*(x-offset)))

def sample(p):
    return random.random()<p

# ASP
# if the vehicle in front of it is too close,
# then decelerate
def prob_asp(p, x, vx, lastAction):
    too_close = sample(logistic(-10, .2, x))

    if (too_close):
        action = env.action_type.actions_indexes["SLOWER"]
    else:
        action = env.action_type.actions_indexes["FASTER"]

    return action


action = env.action_type.actions_indexes["IDLE"]
for _ in range(1000):
    obs, reward, done, truncated, info = env.step(action)
    env.render()
    print(obs)
    p=obs[1][0]
    x=obs[1][1]
    vx=obs[1][2]
    action = prob_asp(p==1, x, vx, action)
