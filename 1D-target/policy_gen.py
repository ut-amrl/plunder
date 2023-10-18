from gym.envs.registration import register
import gym
import csv
import numpy as np
import sys
import time
import random

sys.path.append("../baselines/gail/")
env_id = "env-1d-v1"
register(id=env_id, entry_point='custom_envs.envs:Env_1d')

env = gym.make(env_id)

env.reset()

# Probabilistic functions
def logistic(offset, slope, x):
    return 1.0/(1.0+np.exp(-slope*(x-offset)))

def logistic2(x, offset, slope):
    return 1.0/(1.0+np.exp(-slope*(x-offset)))

def sample(p):
    return random.random()<p

def Plus(x, y):
    return x + y

def Abs(x):
    return abs(x)

def Minus(x, y):
    return x - y

def And(x, y):
    return x and y

def Or(x, y):
    return x or y

def Lt(x, y):
    return x < y

def Gt(x, y):
    return x > y

def DistTraveled(v, dec):
    return - v * v / (2 * dec)

def asp(observation, ha) -> str:
    pos, decMax, accMax, vMax, v, acc, ACC, CON, DEC = observation[0], observation[1], observation[2], observation[3], observation[4], observation[5], observation[6], observation[7], observation[8]
    dns = 100 - pos
    target = 100
    
    # Ground Truth
    # cond1 = sample(logistic(0, 1, v - vMax))
    # cond2 = sample(logistic(100, 0.5, pos + DistTraveled(v, decMax)))
    
    # if ha == "ACC":
    #     if cond1 and not cond2:
    #         return "CON"
    #     elif cond2:
    #         return "DEC"
    # elif ha == "CON":
    #     if cond2:
    #         return "DEC"
    
    # PLUNDER
    if ha == "ACC" and sample(logistic2(Minus(v, vMax), -0.456573, 1.388253)):
        return "CON"
    if ha == "ACC" and sample(logistic2(Minus(dns, DistTraveled(v, decMax)), -6.926392, -2.609171)):
        return "DEC"
    if ha == "CON" and sample(logistic2(Minus(DistTraveled(v, decMax), dns), -2.829544, 0.814321)):
        return "DEC"

    # OneShot
    # if ha == "ACC" and sample(logistic2(Minus(vMax, v), -8.960038, -0.226322)):
    #     return "CON"
    # if ha == "ACC" and sample(logistic2(pos, 70.157356, 0.085864)):
    #     return "DEC"
    # if ha == "CON" and sample(logistic2(Minus(v, vMax), -36.486195, -0.148037)):
    #     return "ACC"
    # if ha == "CON" and sample(logistic2(pos, 117.538284, 0.057660)):
    #     return "DEC"
    # if ha == "DEC" and sample(logistic2(pos, -61.554161, -0.039673)):
    #     return "ACC"
    # if ha == "DEC" and sample(logistic2(decMax, 42.312191, 0.113730)):
    #     return "CON"

    # Greedy
    # if ha == "ACC" and sample(logistic2(Minus(v, vMax), 14.627029, 0.054267)):
    #     return "CON"
    # if ha == "ACC" and sample(logistic2(pos, 53.392311, 0.031613)):
    #     return "DEC"
    # if ha == "CON" and sample(logistic2(pos, 2.379374, -0.020114)):
    #     return "ACC"
    # if ha == "CON" and sample(logistic2(pos, 56.861023, 0.023528)):
    #     return "DEC"
    # if ha == "DEC" and sample(logistic2(pos, -116.017990, -0.006218)):
    #     return "ACC"
    # if ha == "DEC" and sample(logistic2(pos, 8.573352, -0.010496)):
    #     return "CON"

    # LDIPS
    # if ha == "ACC" and Gt(Minus(DistTraveled(v, decMax), DistTraveled(vMax, decMax)), 2.149704):
    #     return "CON"
    # if ha == "ACC" and Gt(Minus(DistTraveled(vMax, accMax), Plus(target, dns)), -199.659897):
    #     return "DEC"
    # if ha == "CON" and Gt(Minus(Plus(dns, dns), DistTraveled(vMax, accMax)), 200.769547):
    #     return "ACC"
    # if ha == "CON" and Gt(Minus(DistTraveled(v, decMax), Plus(target, dns)), -102.982086):
    #     return "DEC"
    # if ha == "DEC" and Gt(Minus(Minus(dns, target), DistTraveled(vMax, accMax)), 32.139507):
    #     return "ACC"
    # if ha == "DEC" and Gt(Minus(Minus(dns, target), DistTraveled(vMax, decMax)), -48.931610):
    #     return "CON"

    return ha

def get_action(observation, ha) -> str:
    pos, decMax, accMax, vMax, vel, acc, ACC, CON, DEC = observation[0], observation[1], observation[2], observation[3], observation[4], observation[5], observation[6], observation[7], observation[8]
    if ha == "ACC":
        return [ACC]
    elif ha == "DEC":
        return [DEC]
    elif ha == "CON":
        return [CON]

success = 0
for _ in range(1000):
    env.config(random.randint(-10, -5), random.randint(5, 10), random.randint(5, 30), 100)
    observation = env.reset()

    ha = "ACC"
    action = [0]

    for _ in range(1000):
        observation, _, _, _ = env.step(action)

        ha = asp(observation, ha)
        action = get_action(observation, ha)

        pos, decMax, accMax, vMax, vel, acc, ACC, CON, DEC = observation[0], observation[1], observation[2], observation[3], observation[4], observation[5], observation[6], observation[7], observation[8]

        if vel < -10:
            break
        if pos > 150:
            break
        if pos < 140 and pos > 60 and abs(vel) < 0.1:
            success += 1
            break

print(success)
env.close()