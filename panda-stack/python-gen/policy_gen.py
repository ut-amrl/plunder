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

######## ASP ########

def asp(observation, ha) -> str:
    x, y, z, end_width = observation[0], observation[1], observation[2], observation[3]
    bx1, by1, bz1, bx2, by2, bz2 = observation[4], observation[5], observation[6], observation[7], observation[8], observation[9]
    tx2, ty2, tz2 = observation[13], observation[14], observation[15]

    # Ground-truth policy
    if ha == 'MOVE_TO_CUBE_BOTTOM' and bz1 > -0.005:
        return 'MOVE_TO_TARGET'
    elif ha == 'MOVE_TO_TARGET' and abs(tx2) + abs(ty2) < 0.003:
        return 'LIFT'
    elif ha == 'LIFT' and sample(logistic(0.15, 200, z)):
        return 'MOVE_TO_CUBE_TOP'
    elif ha == 'MOVE_TO_CUBE_TOP' and bz2 > -0.005:
        return 'GRASP'
    elif ha == 'GRASP' and sample(logistic(0.15, 200, z)):
        return 'MOVE_TO_TARGET'
    return ha

    # PLUNDER-synthesized ASP
    # if ha == 'GRASP' and sample(logistic(-0.100266, -80.894936, tz2)):
    #     return 'MOVE_TO_TARGET'
    # elif ha == 'LIFT' and sample(logistic(0.6139, 41.856, bz1 + ty2)):
    #     return 'MOVE_TO_CUBE_BOTTOM'
    # elif ha == 'LIFT' and sample(logistic(-0.142514, -103.338959, bz1)):
    #     return 'MOVE_TO_CUBE_TOP'
    # elif ha == 'MOVE_TO_CUBE_BOTTOM' and sample(logistic(0.4281, 116.65596, ty2)):
    #     return 'LIFT'
    # elif ha == 'MOVE_TO_CUBE_BOTTOM' and sample(logistic(-0.332, -20.043522, z)):
    #     return 'MOVE_TO_CUBE_TOP'
    # elif ha == 'MOVE_TO_CUBE_BOTTOM' and sample(logistic(0.02998, 29400.211, bz1 + tz2)):
    #     return 'MOVE_TO_TARGET'
    # elif ha == 'MOVE_TO_CUBE_TOP' and sample(logistic(-0.009927, 83562.8, bz2 + bz2)):
    #     return 'GRASP'
    # elif ha == 'MOVE_TO_CUBE_TOP' and sample(logistic(0.176457, 44.735584, tz2)):
    #     return 'MOVE_TO_CUBE_BOTTOM'
    # elif ha == 'MOVE_TO_TARGET' and sample(logistic(0.002998, -41319.92, abs(tx2) + abs(ty2))):
    #     return 'LIFT'
    # return ha

    # OneShot synthesized ASP
    # if ha == "GRASP" and sample(logistic2(Plus(Abs(tx2), Abs(ty2)), 0.002903, -8579.819336)):
    #     return "LIFT"
    # if ha == "GRASP" and sample(logistic2(bz2, 0.048008, 560.119995)):
    #     return "MOVE_TO_CUBE_BOTTOM"
    # if ha == "GRASP" and sample(logistic2(Abs(ty2), -0.000712, -1391.278442)):
    #     return "MOVE_TO_CUBE_TOP"
    # if ha == "GRASP" and sample(logistic2(tz2, -0.127233, -29.926411)):
    #     return "MOVE_TO_TARGET"
    # if ha == "LIFT" and sample(logistic2(bz2, -0.004620, 3596.524902)):
    #     return "GRASP"
    # if ha == "LIFT" and sample(logistic2(Plus(z, tx2), 0.279324, 29.211910)):
    #     return "MOVE_TO_CUBE_BOTTOM"
    # if ha == "LIFT" and sample(logistic2(tz2, -0.121193, -45.976688)):
    #     return "MOVE_TO_CUBE_TOP"
    # if ha == "LIFT" and sample(logistic2(z, 0.004615, -88.014793)):
    #     return "MOVE_TO_TARGET"
    # if ha == "MOVE_TO_CUBE_BOTTOM" and sample(logistic2(bz2, -0.003502, 1726.153442)):
    #     return "GRASP"
    # if ha == "MOVE_TO_CUBE_BOTTOM" and sample(logistic2(y, -0.494754, -11.371115)):
    #     return "LIFT"
    # if ha == "MOVE_TO_CUBE_BOTTOM" and sample(logistic2(Abs(bx2), -0.041487, -52.337048)):
    #     return "MOVE_TO_CUBE_TOP"
    # if ha == "MOVE_TO_CUBE_BOTTOM" and sample(logistic2(Plus(z, Abs(by1)), 0.026221, -528.714661)):
    #     return "MOVE_TO_TARGET"
    # if ha == "MOVE_TO_CUBE_TOP" and sample(logistic2(bz2, 0.001076, 243.634109)):
    #     return "GRASP"
    # if ha == "MOVE_TO_CUBE_TOP" and sample(logistic2(Plus(z, bz1), 0.022040, 2916.803467)):
    #     return "LIFT"
    # if ha == "MOVE_TO_CUBE_TOP" and sample(logistic2(bz2, 0.250416, 15.667148)):
    #     return "MOVE_TO_CUBE_BOTTOM"
    # if ha == "MOVE_TO_CUBE_TOP" and sample(logistic2(bz2, -0.004936, 19926.169922)):
    #     return "MOVE_TO_TARGET"
    # if ha == "MOVE_TO_TARGET" and sample(logistic2(tx2, -0.204121, -27.988668)):
    #     return "GRASP"
    # if ha == "MOVE_TO_TARGET" and sample(logistic2(Plus(Abs(tx2), Abs(ty2)), 0.002917, -13702.009766)):
    #     return "LIFT"
    # if ha == "MOVE_TO_TARGET" and sample(logistic2(by2, 0.597291, 14.153955)):
    #     return "MOVE_TO_CUBE_BOTTOM"
    # if ha == "MOVE_TO_TARGET" and sample(logistic2(Abs(tx2), -0.000444, -1469.258423)):
    #     return "MOVE_TO_CUBE_TOP"
    # return ha

    # Greedy
    # if ha == "GRASP" and sample(logistic2(Plus(Abs(tx2), Abs(ty2)), 0.002881, -19521.787109)):
    #     return "LIFT"
    # if ha == "GRASP" and sample(logistic2(bz1, 1000000000.000000, 1.000000)):
    #     return "MOVE_TO_CUBE_BOTTOM"
    # if ha == "GRASP" and sample(logistic2(Plus(bz2, Abs(tx2)), -0.048223, -31575.359375)):
    #     return "MOVE_TO_CUBE_TOP"
    # if ha == "GRASP" and sample(logistic2(Abs(bx1), 0.070747, -21.158556)):
    #     return "MOVE_TO_TARGET"
    # if ha == "LIFT" and sample(logistic2(tz2, 0.035738, 2132.302002)):
    #     return "GRASP"
    # if ha == "LIFT" and sample(logistic2(tz2, 0.036302, 13.266532)):
    #     return "MOVE_TO_CUBE_BOTTOM"
    # if ha == "LIFT" and sample(logistic2(Abs(by1), 0.022663, 63.118954)):
    #     return "MOVE_TO_CUBE_TOP"
    # if ha == "LIFT" and sample(logistic2(tz2, 0.034231, 16233.675781)):
    #     return "MOVE_TO_TARGET"
    # if ha == "MOVE_TO_CUBE_BOTTOM" and sample(logistic2(y, -0.256241, -23.163843)):
    #     return "GRASP"
    # if ha == "MOVE_TO_CUBE_BOTTOM" and sample(logistic2(bx1, -0.308033, -4.828975)):
    #     return "LIFT"
    # if ha == "MOVE_TO_CUBE_BOTTOM" and sample(logistic2(Abs(bx2), 0.017093, -23.187845)):
    #     return "MOVE_TO_CUBE_TOP"
    # if ha == "MOVE_TO_CUBE_BOTTOM" and sample(logistic2(Abs(by1), -0.004330, -216.002945)):
    #     return "MOVE_TO_TARGET"
    # if ha == "MOVE_TO_CUBE_TOP" and sample(logistic2(bz2, -0.004638, 3795.937744)):
    #     return "GRASP"
    # if ha == "MOVE_TO_CUBE_TOP" and sample(logistic2(y, 0.474148, 3.911703)):
    #     return "LIFT"
    # if ha == "MOVE_TO_CUBE_TOP" and sample(logistic2(bz2, 0.043619, 10.446322)):
    #     return "MOVE_TO_CUBE_BOTTOM"
    # if ha == "MOVE_TO_CUBE_TOP" and sample(logistic2(tz2, 0.035082, 69877.320312)):
    #     return "MOVE_TO_TARGET"
    # if ha == "MOVE_TO_TARGET" and sample(logistic2(by2, -0.893835, -1.632642)):
    #     return "GRASP"
    # if ha == "MOVE_TO_TARGET" and sample(logistic2(Plus(Abs(tx2), Abs(ty2)), 0.002860, -7998.380859)):
    #     return "LIFT"
    # if ha == "MOVE_TO_TARGET" and sample(logistic2(by2, 0.510715, 10.604516)):
    #     return "MOVE_TO_CUBE_BOTTOM"
    # if ha == "MOVE_TO_TARGET" and sample(logistic2(Abs(tx2), -0.000544, -1162.145996)):
    #     return "MOVE_TO_CUBE_TOP"
    # return ha

    # LDIPS
    # if ha == "GRASP" and Lt(Plus(Abs(tx2), Abs(ty2)), 0.003036):
    #     return "LIFT"
    # if ha == "GRASP" and Gt(bx1, 0.239810):
    #     return "MOVE_TO_CUBE_BOTTOM"
    # if ha == "GRASP" and Lt(Plus(Abs(tx2), Abs(ty2)), 0.003036):
    #     return "MOVE_TO_CUBE_TOP"
    # if ha == "GRASP" and Gt(Plus(z, bx2), 0.171815):
    #     return "MOVE_TO_TARGET"
    # if ha == "LIFT" and Gt(tz2, 0.035169):
    #     return "GRASP"
    # if ha == "LIFT" and Gt(Plus(Abs(tx2), Abs(tx2)), 0.385529):
    #     return "MOVE_TO_CUBE_BOTTOM"
    # if ha == "LIFT" and Gt(Plus(z, Abs(bx1)), 0.175868):
    #     return "MOVE_TO_CUBE_TOP"
    # if ha == "LIFT" and Gt(Plus(bz2, bz2), -0.010058):
    #     return "MOVE_TO_TARGET"
    # if ha == "MOVE_TO_CUBE_BOTTOM" and Lt(Plus(Abs(bx2), Abs(bz2)), 0.006300):
    #     return "GRASP"
    # if ha == "MOVE_TO_CUBE_BOTTOM" and Gt(by2, 0.196789):
    #     return "LIFT"
    # if ha == "MOVE_TO_CUBE_BOTTOM" and Lt(Abs(ty2), 0.001498):
    #     return "MOVE_TO_CUBE_TOP"
    # if ha == "MOVE_TO_CUBE_BOTTOM" and Gt(Plus(bz2, bz2), -0.010051):
    #     return "MOVE_TO_TARGET"
    # if ha == "MOVE_TO_CUBE_TOP" and Gt(Plus(bz2, bz2), -0.009983):
    #     return "GRASP"
    # if ha == "MOVE_TO_CUBE_TOP" and Gt(Plus(bz1, Abs(bz2)), 0.004851):
    #     return "LIFT"
    # if ha == "MOVE_TO_CUBE_TOP" and Lt(Plus(tx2, Abs(y)), -0.038787):
    #     return "MOVE_TO_CUBE_BOTTOM"
    # if ha == "MOVE_TO_CUBE_TOP" and Gt(bz2, -0.005009):
    #     return "MOVE_TO_TARGET"
    # if ha == "MOVE_TO_TARGET" and Gt(Plus(bz1, Abs(bx1)), 0.119720):
    #     return "GRASP"
    # if ha == "MOVE_TO_TARGET" and Lt(Plus(Abs(tx2), Abs(ty2)), 0.003033):
    #     return "LIFT"
    # if ha == "MOVE_TO_TARGET" and Gt(by2, 0.197014):
    #     return "MOVE_TO_CUBE_BOTTOM"
    # if ha == "MOVE_TO_TARGET" and Lt(Plus(Abs(tx2), Abs(ty2)), 0.003033):
    #     return "MOVE_TO_CUBE_TOP"
    # return ha

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
            action[2] = max(past_action[2] - 0.15, action[2])
    elif ha == 'LIFT':
        action = [0, 0, 0.5, 1]
    elif ha == 'MOVE_TO_CUBE_TOP':
        action = [bx2 * 4.0, by2 * 4.0, bz2 * 4.0, 1]
        if action[2] < 0:
            action[2] = max(past_action[2] - 0.15, action[2])
    elif ha == 'GRASP':
        action = [0, 0, 0.5, -1]

    return action

env = gym.make("PandaStackDense-v3", render_mode="human")

def bound(x):
    return max(min(x, 1), -1)

success = 0
for iter in range(100):
    obs_out = open("data" + str(iter) + ".csv", "w")
    obs_out.write("x, y, z, end_width, bx1, by1, bz1, bx2, by2, bz2, tx1, ty1, tz1, tx2, ty2, tz2, LA.vx, LA.vy, LA.vz, LA.end, HA\n")

    observation, info = env.reset()

    ha = 'MOVE_TO_CUBE_BOTTOM'
    action = [0, 0, 0, 0]

    for _ in range(125):
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

        if terminated:
            success += 1
            break
    
    obs_out.close()

print(success)
env.close()