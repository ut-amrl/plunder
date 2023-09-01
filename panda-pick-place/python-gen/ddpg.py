import gymnasium as gym
import panda_gym
from stable_baselines3 import DDPG
import time

env = gym.make("PandaPickAndPlaceDense-v3")
model = DDPG(policy="MultiInputPolicy", env=env)
model.learn(100_000)

model.save("ddpg-pick-and-place")


env = gym.make("PandaPickAndPlaceDense-v3", render_mode="human")
for iter in range(50):
    obs_out = open("data" + str(iter) + ".csv", "w")
    obs_out.write("x, y, z, bx, by, bz, tx, ty, tz, end_width, LA.vx, LA.vy, LA.vz, LA.end, HA\n")

    observation, info = env.reset()

    action = [0, 0, 0, 0]

    for _ in range(50):
        observation, reward, terminated, truncated, info = env.step(action)
        action, _states = model.predict(observation, deterministic=True)
        
        world_state = observation["observation"]
        target_pos = observation["desired_goal"][0:3]

        x, y, z, bx, by, bz, tx, ty, tz, end_width = world_state[0], world_state[1], world_state[2], world_state[7], world_state[8], world_state[9], target_pos[0], target_pos[1], target_pos[2], world_state[6]
        bx, by, bz = bx - x, by - y, bz - z
        tx, ty, tz = tx - x, ty - y, tz - z

        obs_pruned = [x, y, z, bx, by, bz, tx, ty, tz, end_width]

        for each in obs_pruned:
            obs_out.write(str(each)+", ")
        for each in action:
            obs_out.write(str(each)+", ")

        obs_out.write("0\n")
        
        time.sleep(0.1)
        if terminated or truncated:
            break
    
    obs_out.close()
    

env.close()