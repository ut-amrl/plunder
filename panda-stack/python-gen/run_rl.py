import gymnasium as gym
import panda_gym
from stable_baselines3 import PPO, DDPG, A2C
from sb3_contrib import TQC
import time
from stable_baselines3.common.evaluation import evaluate_policy
import numpy as np

algo = "TQC"

env = gym.make("PandaStackDense-v3", render_mode="human")

if algo == "DDPG":
    model = DDPG.load("ddpg-stack", env=env)

elif algo == "A2C":
    model = A2C.load("a2c-stack", env=env)

elif algo == "PPO":
    model = PPO.load("ppo-stack", env=env)

elif algo == "TQC":
    model = TQC.load("tqc-stack", env=env)

mean_reward, std_reward = evaluate_policy(model, env, deterministic=True, render=True)

print(f"Mean reward = {mean_reward:.2f} +/- {std_reward:.2f}")

for iter in range(15):
    obs_out = open("data" + str(iter) + ".csv", "w")
    obs_out.write("x, y, z, end_width, bx1, by1, bz1, bx2, by2, bz2, tx1, ty1, tz1, tx2, ty2, tz2, LA.vx, LA.vy, LA.vz, LA.end, HA\n")

    observation, info = env.reset()

    action = [0, 0, 0, 0]

    for _ in range(150):
        observation, reward, terminated, truncated, info = env.step(action)
        action, _states = model.predict(observation, deterministic=True)
        
        world_state = observation["observation"]
        target_bottom = observation["desired_goal"][0:3]
        target_top = observation["desired_goal"][3:6]

        x, y, z, end_width = world_state[0], world_state[1], world_state[2], world_state[6]
        bx1, by1, bz1, bx2, by2, bz2 = world_state[7], world_state[8], world_state[9], world_state[19], world_state[20], world_state[21]
        tx1, ty1, tz1, tx2, ty2, tz2 = target_bottom[0], target_bottom[1], target_bottom[2], target_top[0], target_top[1], target_top[2]
        obs_pruned = [x, y, z, end_width, bx1, by1, bz1, bx2, by2, bz2, tx1, ty1, tz1, tx2, ty2, tz2]

        for each in obs_pruned:
            obs_out.write(str(each)+", ")
        for each in action:
            obs_out.write(str(each)+", ")

        obs_out.write("0\n")
        
        time.sleep(0.05)
    
    obs_out.close()
    
env.close()