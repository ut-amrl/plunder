import gymnasium as gym
import panda_gym
import time
import numpy as np

def asp(observation, ha) -> str:
    x, y, z, bx, by, bz, tx, ty, tz, end_width = observation[0], observation[1], observation[2], observation[3], observation[4], observation[5], observation[6], observation[7], observation[8], observation[9]
    if ha == "MOVE_TO_CUBE" and (bz - (z + abs(z))) - (abs(z) - abs(ty)) > -0.024638: # PLUNDER-synthesized transition condition
        return "MOVE_TO_TARGET"
    return ha

def get_action(observation, ha) -> str:
    x, y, z, bx, by, bz, tx, ty, tz, end_width = observation[0], observation[1], observation[2], observation[3], observation[4], observation[5], observation[6], observation[7], observation[8], observation[9]
    bx, by, bz = bx - x, by - y, bz - z
    tx, ty, tz = tx - x, ty - y, tz - z
    if ha == 'MOVE_TO_CUBE':
        return [bx * 5.0, by * 5.0, bz * 5.0, 1]
    else:
        return [tx * 5.0, ty * 5.0, tz * 5.0, -1]

env = gym.make("PandaPickAndPlace-v3", render_mode="human")

for iter in range(30):
    # obs_out = open("data" + str(iter) + ".csv", "w")
    # obs_out.write("x, y, z, bx, by, bz, tx, ty, tz, end_width, LA.vx, LA.vy, LA.vz, LA.end, HA\n")

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
        action = get_action(obs_pruned, ha)

        # for each in obs_pruned:
        #     obs_out.write(str(each)+", ")
        # for each in action:
        #     obs_out.write(str(each)+", ")

        # if ha == 'MOVE_TO_CUBE':
        #     obs_out.write("0\n")
        # elif ha == 'GRASP':
        #     obs_out.write("1\n")
        # elif ha == 'MOVE_TO_TARGET':
        #     obs_out.write("2\n")
        
        time.sleep(0.08)
        if terminated or truncated:
            break
    
    # obs_out.close()
    

env.close()