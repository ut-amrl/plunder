import gymnasium as gym
import panda_gym
import time
import numpy as np
import random

######## ASP ########
# Probabilistic functions
def logistic(offset, slope, x):
    return 1.0/(1.0+np.exp(-slope*(x-offset)))

def sample(p):
    return random.random()<p

def asp(observation, ha) -> str:
    x, y, z, end_width = observation[0], observation[1], observation[2], observation[3]
    bx1, by1, bz1, bx2, by2, bz2 = observation[4], observation[5], observation[6], observation[7], observation[8], observation[9]
    tx2, ty2, tz2 = observation[13], observation[14], observation[15]

    if ha == 'MOVE_TO_CUBE_BOTTOM' and bz1 > -0.002:
        return 'MOVE_TO_TARGET'
    elif ha == 'MOVE_TO_TARGET' and abs(tx2) < 0.002 and abs(ty2) < 0.002:
        return 'LIFT'
    elif ha == 'LIFT' and sample(logistic(0.1, 100, z)) and bz2 - bz1 < 0.03:
        return 'MOVE_TO_CUBE_TOP'
    elif ha == 'MOVE_TO_CUBE_TOP' and bz2 > -0.002:
        return 'GRASP'
    elif ha == 'GRASP' and sample(logistic(0.1, 100, z)):
        return 'MOVE_TO_TARGET'
    return ha

def get_action(observation, past_action, ha) -> str:
    x, y, z, end_width = observation[0], observation[1], observation[2], observation[3]
    bx1, by1, bz1, bx2, by2, bz2 = observation[4], observation[5], observation[6], observation[7], observation[8], observation[9]
    tx2, ty2, tz2 = observation[13], observation[14], observation[15]

    tz2 += 0.01

    if ha == 'MOVE_TO_CUBE_BOTTOM':
        action = [bx1 * 4.0, by1 * 4.0, bz1 * 4.0, 1]
    elif ha == 'MOVE_TO_TARGET':
        action = [tx2 * 4.0, ty2 * 4.0, tz2 * 4.0, -1]
        if action[2] < 0:
            action[2] = max(past_action[2] - 0.05, action[2])
    elif ha == 'LIFT':
        action = [0, 0, 0.5, 1]
    elif ha == 'MOVE_TO_CUBE_TOP':
        action = [bx2 * 4.0, by2 * 4.0, bz2 * 4.0, 1]
        if action[2] < 0:
            action[2] = max(past_action[2] - 0.05, action[2])
    elif ha == 'GRASP':
        action = [0, 0, 0.5, -1]

    return action

env = gym.make("PandaStackDense-v3", render_mode="human")

def bound(x):
    return max(min(x, 1), -1)

for iter in range(20):
    obs_out = open("data" + str(iter) + ".csv", "w")
    obs_out.write("x, y, z, end_width, bx1, by1, bz1, bx2, by2, bz2, tx1, ty1, tz1, tx2, ty2, tz2, LA.vx, LA.vy, LA.vz, LA.end, HA\n")

    observation, info = env.reset()

    ha = 'MOVE_TO_CUBE_BOTTOM'
    action = [0, 0, 0, 0]

    for _ in range(175):
        observation, reward, terminated, truncated, info = env.step(action)

        world_state = observation["observation"]
        target_bottom = observation["desired_goal"][0:3]
        target_top = observation["desired_goal"][3:6]

        x, y, z, end_width = world_state[0], world_state[1], world_state[2], world_state[6]
        bx1, by1, bz1, bx2, by2, bz2 = world_state[7], world_state[8], world_state[9], world_state[19], world_state[20], world_state[21]
        tx1, ty1, tz1, tx2, ty2, tz2 = target_bottom[0], target_bottom[1], target_bottom[2], target_top[0], target_top[1], target_top[2]

        bx1, by1, bz1, bx2, by2, bz2 = bx1 - x, by1 - y, bz1 - z, bx2 - x, by2 - y, bz2 - z
        tx2, ty2, tz2 = tx2 - x, ty2 - y, tz2 - z

        obs_pruned = [x, y, z, end_width, bx1, by1, bz1, bx2, by2, bz2, tx1, ty1, tz1, tx2, ty2, tz2]
        ha = asp(obs_pruned, ha)
        action = get_action(obs_pruned, action, ha)
        
        for each in obs_pruned:
            obs_out.write(str(each)+", ")
        for each in action:
            with_err = bound(np.random.normal(each, 0.3))
            obs_out.write(str(with_err)+", ")

        if ha == 'MOVE_TO_CUBE_BOTTOM':
            obs_out.write("0\n")
        elif ha == 'MOVE_TO_TARGET':
            obs_out.write("1\n")
        elif ha == 'LIFT':
            obs_out.write("2\n")
        elif ha == 'MOVE_TO_CUBE_TOP':
            obs_out.write("3\n")
        elif ha == 'GRASP':
            obs_out.write("4\n")
        
        time.sleep(0.05)
        # if terminated or truncated:
        #     break
    
    obs_out.close()

env.close()