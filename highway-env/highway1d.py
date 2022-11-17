import gym
import highway_env
from matplotlib import pyplot as plt
import os
import math
import random

env = gym.make('highway-v0')
env.config['lanes_count']=1
env.config['vehicles_count']=2
env.config['simulation_frequency']=100
env.config['policy_frequency']=10
env.config['observation']={
    'type': 'Kinematics',
    'vehicles_count': 3,
    'features': ['presence', 'x', 'vx'],
    'absolute': False
}

highway_env.highway_env.envs.ControlledVehicle.DELTA_SPEED = 15
env.reset()

# observations
# ego vehicle:  presence, x, vx
# vehicle 1:    presence, x, vx        (relative to ego)
# vehicle 2:    presence, x, vx        (relative to ego)

# ASP
# if either the vehicle in front of it is too close,
# or the vehicle in front of it is relatively close and decelerating,
# then decelerate
# if (vx<0 and x<.2) or (x<.1) then SLOWER

def logistic(slope, offset, x):
    return 1.0/(1.0+pow(math.e, -slope*(x-offset)))

def sample(p):
    return random.random()<p

def prob_asp(p1, p2, x1, x2, vx1, vx2):
    rel_close1 = sample(logistic(-40, .2, x1))
    # rel_close2 = sample(logistic(-40, .2, x2))

    # too_close1 = sample(logistic(-40, .1, x1))
    # too_close2 = sample(logistic(-40, .1, x2))

    # too_fast1 = sample(logistic(-20, 0, vx1))
    # too_fast2 = sample(logistic(-20, 0, vx2))

    action=0
    if (p1 and rel_close1):
        action = env.action_type.actions_indexes["SLOWER"]
    else:
        action = env.action_type.actions_indexes["FASTER"]
    return action


action = env.action_type.actions_indexes["IDLE"]
for _ in range(1000):
    obs, reward, done, truncated, info = env.step(action)
    env.render()
    print(obs)
    p1=obs[1][0]
    p2=obs[2][0]
    x1=obs[1][1]
    x2=obs[2][1]
    vx1=obs[1][2]
    vx2=obs[2][2]
    action = prob_asp(p1==1, p2==1, x1, x2, vx1, vx2)

plt.imshow(env.render(mode="rgb_array"))
plt.show()
