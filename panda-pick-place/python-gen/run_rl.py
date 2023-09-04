import gymnasium as gym
import panda_gym
from stable_baselines3 import PPO, DDPG, A2C
from sb3_contrib import TQC
import time
from stable_baselines3.common.evaluation import evaluate_policy
import numpy as np

algo = "TQC"

env = gym.make("PandaPickAndPlaceDense-v3", render_mode="human")

if algo == "DDPG":
    model = DDPG.load("ddpg-pick-and-place", env=env)

elif algo == "A2C":
    model = A2C.load("a2c-pick-and-place", env=env)

elif algo == "PPO":
    model = PPO.load("ppo-pick-and-place", env=env)

elif algo == "TQC":
    model = TQC.load("tqc-pick-and-place", env=env)

mean_reward, std_reward = evaluate_policy(model, env, deterministic=True, render=True)

print(f"Mean reward = {mean_reward:.2f} +/- {std_reward:.2f}")

obs_stddev = 0.01

for iter in range(30):
    obs_out = open("data" + str(iter) + ".csv", "w")
    obs_out.write("x, y, z, bx, by, bz, tx, ty, tz, end_width, LA.vx, LA.vy, LA.vz, LA.end, HA\n")

    observation, info = env.reset()

    action = [0, 0, 0, 0]
    solved = False

    for _ in range(30):
        observation, reward, terminated, truncated, info = env.step(action)
        action, _states = model.predict(observation, deterministic=True)
        if solved:
            action = [0, 0, 0, -1]
        
        world_state = observation["observation"]
        target_pos = observation["desired_goal"][0:3]

        x, y, z, bx, by, bz, tx, ty, tz, end_width = world_state[0], world_state[1], world_state[2], world_state[7], world_state[8], world_state[9], target_pos[0], target_pos[1], target_pos[2], world_state[6]
        obs_pruned = [x, y, z, bx, by, bz, tx, ty, tz, end_width]

        for each in obs_pruned:
            with_err = np.random.normal(each, obs_stddev)
            obs_out.write(str(each)+", ")
        for each in action:
            obs_out.write(str(each)+", ")

        obs_out.write("0\n")
        
        time.sleep(0.08)
        if terminated or truncated:
            solved = True
    
    obs_out.close()
    

env.close()