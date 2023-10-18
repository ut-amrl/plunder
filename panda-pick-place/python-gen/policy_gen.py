import gymnasium as gym
import panda_gym
import time
import numpy as np
import random

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

def asp(observation, ha) -> str:
    x, y, z, bx, by, bz, tx, ty, tz, end_width = observation[0], observation[1], observation[2], observation[3], observation[4], observation[5], observation[6], observation[7], observation[8], observation[9]

    # if ha == "MOVE_TO_CUBE" and sample(logistic(0.007, -5000, abs(x - bx))) and sample(logistic(0.007, -5000, abs(y - by))):
    #     return "MOVE_TO_TARGET"
    
    # RL-based

    # if ha == "MOVE_TO_CUBE" and (sample(logistic2(x, -0.091224, -353.972)) or sample(logistic2(z, 0.02349, -2419.06))):
    #     return "MOVE_TO_TARGET"
    
    # OneShot
    # if ha == "MOVE_TO_CUBE" and sample(logistic2(z, 0.032951, -96.541489)): # PLUNDER
    #     return "MOVE_TO_TARGET"
    # elif ha == "MOVE_TO_TARGET" and sample(logistic2(tz, -0.054478, -70.535027)):
    #     return "MOVE_TO_CUBE"
    
    # Greedy
    # if ha == "MOVE_TO_CUBE" and sample(logistic2(Minus(z, bz), 0.029545, -41.118660)):
    #     return "MOVE_TO_TARGET"
    # elif ha == "MOVE_TO_TARGET" and sample(logistic2(tz, -0.126017, -14.454493)):
    #     return "MOVE_TO_CUBE"

    # LDIPS
    if ha == "MOVE_TO_CUBE" and (z < 0.044347 or abs(y) > 0.097483):
        return "MOVE_TO_TARGET"
    elif ha == "MOVE_TO_TARGET" and Minus(Abs(bx), Abs(x)) > 0.030193:
        return "MOVE_TO_CUBE"


    # Policy-based
    # PLUNDER
    # if ha == "MOVE_TO_CUBE" and sample(logistic2(Minus(Minus(Abs(x), Abs(bx)), Plus(Minus(z, bz), Minus(Abs(Minus(z, Abs(bx))), Abs(x)))), 0.011845, 539.856995)):
        # return "MOVE_TO_TARGET"

    # OneShot
    # if ha == "MOVE_TO_CUBE" and sample(logistic2(Minus(x, bx), -0.004424, 341.041840)) and sample(logistic2(z, 0.002698, -184.910858)):
    #     return "MOVE_TO_TARGET"
    # elif ha == "MOVE_TO_TARGET" and sample(logistic2(Minus(y, bx), -0.265226, -31.038519)):
    #     return "MOVE_TO_CUBE"
    
    # Greedy
    # if ha == "MOVE_TO_CUBE" and sample(logistic2(bz, 0.028010, 202.720947)):
    #     return "MOVE_TO_TARGET"
    # elif ha == "MOVE_TO_TARGET" and sample(logistic2(Minus(Abs(bx), Abs(x)), 0.008767, 518.401184)):
    #     return "MOVE_TO_CUBE"

    # LDIPS
    # if ha == "MOVE_TO_CUBE" and x + abs(bx) < 0.006822 or x > 0.099304:
    #     return "MOVE_TO_TARGET"
    # elif ha == "MOVE_TO_TARGET" and abs(bx) - abs(x) > 0.005853:
    #     return "MOVE_TO_CUBE"

    return ha

def return_closer(next, cur):
    if next > cur:
        return min(next, cur + 0.4)
    return max(next, cur - 0.4)

def get_action(observation, past_action, ha) -> str:
    x, y, z, bx, by, bz, tx, ty, tz, end_width = observation[0], observation[1], observation[2], observation[3], observation[4], observation[5], observation[6], observation[7], observation[8], observation[9]
    bx, by, bz = bx - x, by - y, bz - z
    tx, ty, tz = tx - x, ty - y, tz - z

    # RL-based
    vx, vy, vz, end = bx * 5, by * 5, bz * 5, 1
    if ha == "MOVE_TO_TARGET":
        vx, vy, vz, end = 5 * tx, 5 * ty, 5 * tz, -1

    vx = return_closer(vx, past_action[0])
    vy = return_closer(vy, past_action[1])
    vz = return_closer(vz, past_action[2])

    return [vx, vy, vz, end]

    # Policy-based
    # if ha == 'MOVE_TO_CUBE':
    #     return [bx * 5.0, by * 5.0, bz * 5.0, 0.6]
    
    # if past_action[3] >= 0.6:
    #     return [bx * 5.0, by * 5.0, bz * 5.0, 0.3]
    # elif past_action[3] >= 0.3:
    #     return [bx * 5.0, by * 5.0, bz * 5.0, 0]
    # elif past_action[3] >= 0:
    #     return [bx * 5.0, by * 5.0, bz * 5.0, -0.3]
    # elif past_action[3] >= -0.3:
    #     return [bx * 5.0, by * 5.0, bz * 5.0, -0.6]
    
    # return [tx * 5.0, ty * 5.0, tz * 5.0, -0.6]

def bound(x):
    return max(min(x, 1), -1)

env = gym.make("PandaPickAndPlace-v3", render_mode="human")

success = 0
for iter in range(500):
    observation, info = env.reset()

    ha = 'MOVE_TO_CUBE'
    action = [0, 0, 0, 0]

    for _ in range(30):
        observation, reward, terminated, truncated, info = env.step(action)

        world_state = observation["observation"]
        target_pos = observation["desired_goal"][0:3]

        x, y, z, bx, by, bz, tx, ty, tz, end_width = world_state[0], world_state[1], world_state[2], world_state[7], world_state[8], world_state[9], target_pos[0], target_pos[1], target_pos[2], world_state[6]

        obs_pruned = [x, y, z, bx, by, bz, tx, ty, tz, end_width]
        ha = asp(obs_pruned, ha)
        action = get_action(obs_pruned, action, ha)

        if terminated:
            success += 1
            break
        
print(success)
env.close()