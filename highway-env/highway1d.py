import gym
import highway_env
from matplotlib import pyplot as plt
import os
# os.environ["SDL_VIDEODRIVER"] = "dummy"

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


action = env.action_type.actions_indexes["IDLE"]
for _ in range(1000):
    obs, reward, done, truncated, info = env.step(action)
    env.render()
    print(obs)
    if (obs[1][2]<0 and obs[1][1]<.2) or (obs[2][2]<0 and obs[2][1]<.2) or (obs[1][1]<.1) or (obs[2][1]<.1):
        action = env.action_type.actions_indexes["SLOWER"]
    else:
        action = env.action_type.actions_indexes["FASTER"]

plt.imshow(env.render(mode="rgb_array"))
plt.show()
