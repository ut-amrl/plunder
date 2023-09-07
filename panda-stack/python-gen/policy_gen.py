import gymnasium as gym
import panda_gym
import time
import numpy as np

def asp(observation, ha) -> str:
    x, y, z, end_width = observation[0], observation[1], observation[2], observation[3]
    bx1, by1, bz1, bx2, by2, bz2 = observation[4], observation[5], observation[6], observation[7], observation[8], observation[9]
    tx1, ty1, tz1, tx2, ty2, tz2 = observation[10], observation[11], observation[12], observation[13], observation[14], observation[15]

    bx1, by1, bz1, bx2, by2, bz2 = bx1 - x, by1 - y, bz1 - z, bx2 - x, by2 - y, bz2 - z
    tx1, ty1, tz1, tx2, ty2, tz2 = tx1 - x, ty1 - y, tz1 - z, tx2 - x, ty2 - y, tz2 - z

    if ha == 'MOVE_TO_CUBE_BOTTOM' and abs(bx1) < 0.002 and abs(by1) < 0.002 and abs(bz1) < 0.005:
        return 'MOVE_TO_TARGET'
    elif ha == 'MOVE_TO_TARGET' and abs(tx1) < 0.002 and abs(ty1) < 0.002:
        return 'LIFT'
    elif ha == 'LIFT' and z > 0.2 and observation[9] < 0.03:
        return 'MOVE_TO_CUBE_TOP'
    elif ha == 'MOVE_TO_CUBE_TOP' and abs(bx2) < 0.003 and abs(by2) < 0.003 and abs(bz2) < 0.005:
        return 'GRASP'
    elif ha == 'GRASP' and z > 0.2:
        return 'MOVE_TO_TARGET'
    return ha

def get_action(observation, ha) -> str:
    x, y, z, end_width = observation[0], observation[1], observation[2], observation[3]
    bx1, by1, bz1, bx2, by2, bz2 = observation[4], observation[5], observation[6], observation[7], observation[8], observation[9]
    tx1, ty1, tz1, tx2, ty2, tz2 = observation[10], observation[11], observation[12], observation[13], observation[14], observation[15]

    bx1, by1, bz1, bx2, by2, bz2 = bx1 - x, by1 - y, bz1 - z, bx2 - x, by2 - y, bz2 - z
    tx1, ty1, tz1, tx2, ty2, tz2 = tx1 - x, ty1 - y, tz1 - z, tx2 - x, ty2 - y, tz2 - z

    if ha == 'MOVE_TO_CUBE_BOTTOM':
        return [bx1 * 5.0, by1 * 5.0, bz1 * 5.0, 1]
    elif ha == 'MOVE_TO_TARGET':
        return [tx2 * 5.0, ty2 * 5.0, tz2 * 5.0, -1]
    elif ha == 'LIFT':
        return [0, 0, 0.5, 1]
    elif ha == 'MOVE_TO_CUBE_TOP':
        return [bx2 * 5.0, by2 * 5.0, bz2 * 5.0, 1]
    elif ha == 'GRASP':
        return [0, 0, 0.5, -1]

env = gym.make("PandaStackDense-v3", render_mode="human")

def bound(x):
    return max(min(x, 1), -1)

for iter in range(15):
    obs_out = open("data" + str(iter) + ".csv", "w")
    obs_out.write("x, y, z, end_width, bx1, by1, bz1, bx2, by2, bz2, tx1, ty1, tz1, tx2, ty2, tz2, LA.vx, LA.vy, LA.vz, LA.end, HA\n")

    observation, info = env.reset()

    ha = 'MOVE_TO_CUBE_BOTTOM'
    action = [0, 0, 0, 0]

    for _ in range(150):
        observation, reward, terminated, truncated, info = env.step(action)

        world_state = observation["observation"]
        target_bottom = observation["desired_goal"][0:3]
        target_top = observation["desired_goal"][3:6]

        x, y, z, end_width = world_state[0], world_state[1], world_state[2], world_state[6]
        bx1, by1, bz1, bx2, by2, bz2 = world_state[7], world_state[8], world_state[9], world_state[19], world_state[20], world_state[21]
        tx1, ty1, tz1, tx2, ty2, tz2 = target_bottom[0], target_bottom[1], target_bottom[2], target_top[0], target_top[1], target_top[2]

        obs_pruned = [x, y, z, end_width, bx1, by1, bz1, bx2, by2, bz2, tx1, ty1, tz1, tx2, ty2, tz2]
        ha = asp(obs_pruned, ha)
        action = get_action(obs_pruned, ha)
        
        for each in obs_pruned:
            with_err = np.random.normal(each, 0.0001)
            obs_out.write(str(with_err)+", ")
        for each in action:
            with_err = bound(np.random.normal(each, 0.3))
            obs_out.write(str(with_err)+", ")

        if ha == 'MOVE_TO_CUBE_BOTTOM':
            obs_out.write("0\n")
        elif ha == 'MOVE_TO_TARGET':
            obs_out.write("1\n")
        elif ha == 'LIFT':
            obs_out.write("2")
        elif ha == 'MOVE_TO_CUBE_TOP':
            obs_out.write("3\n")
        elif ha == 'GRASP':
            obs_out.write("4\n")
        
        time.sleep(0.05)
        # if terminated or truncated:
        #     break
    
    obs_out.close()

env.close()