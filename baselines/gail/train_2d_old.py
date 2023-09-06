import gym
from gym.envs.registration import register
import numpy as np
from typing import List, Tuple, Union, Optional
from highway_env.envs import MDPVehicle, ControlledVehicle, Vehicle
import random

register(
        id='highway-custom',
        entry_point='custom_envs.envs:HighwayEnv',
    )

train_model = True
test_gt = False
lanes_count = 4
lane_diff = 4
dataPath = "highway-2d-low-error"

steer_err = 0.004
acc_err = .01

# ----------------------------------------------------------------------------------------
rng = np.random.default_rng(0)
config={}
config['simulation_frequency']=20
config['policy_frequency']=5 # Runs once every 4 simulation steps
config['lanes_count']=lanes_count
config['observation']={
    'type': 'CustomObservation',
    'vehicles_count': 10,
    'features': ['presence', 'x', 'y', 'vx', 'vy', 'heading'],
    'absolute': False,
    'normalize': False
}
config['collision_reward']=-10
config['right_lane_reward']=0
config['high_speed_reward']=0
config['on_road_reward']=1
config['reward_speed_range']=[-20,-10]
config['normalize_reward']=False
config['action']={
  'type': 'ContinuousAction'
}
config['duration']=50
env = gym.make("highway-custom", config=config)

# -------------

######## ASP ########
# Probabilistic functions
def logistic(offset, slope, x):
    return 1.0/(1.0+np.exp(-slope*(x-offset)))

def sample(p):
    return random.random()<p

# Helper functions

# Round y-positions to the nearest lane
def laneFinder(y):
    return round(y / lane_diff)


# ASP (probabilistic)
def prob_asp(ego, closest):
    front_clear = sample(logistic(30, 1, closest[1][0]))
    left_clear = sample(logistic(30, 1, closest[0][0]))
    right_clear = sample(logistic(30, 1, closest[2][0]))

    # Deterministic version
    # front_clear = closest[1][1] > 30
    # left_clear = closest[0][1] > 30
    # right_clear = closest[2][1] > 30

    if front_clear: # No car in front: accelerate
        return "FASTER"
    if left_clear: # No car on the left: merge left
        return "LANE_LEFT"
    if right_clear: # No car on the right: merge right
        return "LANE_RIGHT"

    # Nowhere to go: decelerate
    return "SLOWER"

KP_A = 0.4 # Jerk constant (higher = faster acceleration)
KP_H = 0.5 # Turning rate
TURN_HEADING = 0.25 # Target heading when turning
TURN_TARGET = 30 # How much to adjust when targeting a lane (higher = smoother)
min_velocity = 16 # Minimum velocity
max_velocity = 30 # Maximum velocity
last_action = "FASTER"

def run_la(vehicle, action: Union[dict, str] = None, step = True) -> None:
    global last_action

    if action == None:
        action = last_action
    last_action = action

    acc = 0.0
    target_heading = 0.0
    if action == "FASTER":
        # Attain max speed
        acc = KP_A * (max_velocity - vehicle.speed)

        # Follow current lane
        target_y = laneFinder(vehicle.position[1]) * lane_diff
        target_heading = np.arctan((target_y - vehicle.position[1]) / TURN_TARGET)
    elif action == "SLOWER":
        # Attain min speed
        acc = KP_A * (min_velocity - vehicle.speed)

        # Follow current lane
        target_y = laneFinder(vehicle.position[1]) * lane_diff
        target_heading = np.arctan((target_y - vehicle.position[1]) / TURN_TARGET)
    elif action == "LANE_RIGHT":
        # Attain rightmost heading
        target_heading = TURN_HEADING
    elif action == "LANE_LEFT":
        # Attain leftmost heading
        target_heading = -TURN_HEADING
 
    la = {"steering": (target_heading - vehicle.heading) * KP_H, "acceleration": acc }

    # Add error
    la['steering'] = np.random.normal(la['steering'], steer_err)
    la['acceleration'] = np.random.normal(la['acceleration'], acc_err)

    if step: # Perform action
        Vehicle.act(vehicle, la)
    
    return [la['acceleration'], la['steering']]

def runSim(iter):
    env.reset()
    ha = "FASTER"
    rew_sum = 0
    for _ in range(200):

        next_la = run_la(env.vehicle, ha, False)
        obs, reward, done, truncated, info = env.step(next_la)
        if(env.vehicle.crashed or not env.vehicle.on_road):
            print("CRASHED")
            break
        rew_sum += reward
        ha = prob_asp(obs[:5], [obs[5:10], obs[10:15], obs[15:20]])

    return rew_sum

if test_gt:
    print("simulation rewards")
    rewards = []
    for iter in range(8):
        rew = runSim(iter)
        rewards.append(rew)
        print(rewards)








# ----------------------------------------------------------------------------------------
import numpy as np
import gym
import highway_env
from stable_baselines3 import PPO
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.ppo import MlpPolicy
import math
import random
import csv

from imitation.algorithms.adversarial.gail import GAIL
from imitation.data import rollout
from imitation.data.wrappers import RolloutInfoWrapper
from imitation.rewards.reward_nets import BasicRewardNet
from imitation.util.networks import RunningNorm
from imitation.util.util import make_vec_env
from imitation.data.types import TrajectoryWithRew


def read_demo(n):
    reader = csv.reader(open(dataPath+"/data"+str(n)+".csv", "r"))
    next(reader)
    traj_obs = []
    traj_acts = []
    for line in reader:
        # LA - [acc, steer]
        traj_acts.append([line[21], line[20]])
        # obs
        res = []
        for v in range(0, 20):
            res.append(line[v])
        traj_obs.append(res)
    return (traj_obs, traj_acts)

def gen_traj(n):
    obs, acts = read_demo(n)
    next_obs = obs[1:]
    next_obs.append([0]*20)
    dones = [False]*(len(obs))
    return {
        'obs': np.array(obs, dtype=float),
        'acts': np.array(acts, dtype=float),
        'next_obs': np.array(next_obs, dtype=float),
        'dones': np.array(dones)
    }

def gen_trajs(n_demos):
    rollouts=[]
    for i in range(n_demos):
        rollouts.append(gen_traj(i))
    return rollouts



rollouts=gen_trajs(8)

venv = make_vec_env("highway-custom", n_envs=8, rng=rng, env_make_kwargs={ 'config': config })
print("observation space : " + str(venv.observation_space))
print("action space : " + str(venv.action_space))






# ----------------------------------------------------------------------------------------
learner = PPO(
    env=venv,
    policy=MlpPolicy,
    batch_size=64,
    ent_coef=0.0,
    learning_rate=0.003,
    n_epochs=50,
)        # PPO = SOTA RL, generator
# MLP = multi layer perceptron (simple NN)

reward_net = BasicRewardNet(
    venv.observation_space,
    venv.action_space,
    normalize_input_layer=RunningNorm,
)                                                 # discriminator
gail_trainer = GAIL(
    demonstrations=rollouts,                       # expert demos
    demo_batch_size=200,
    gen_replay_buffer_capacity=400,
    n_disc_updates_per_round=4,
    venv=venv,                                     # environment
    gen_algo=learner,
    reward_net=reward_net,
    allow_variable_horizon=True
)







# ----------------------------------------------------------------------------------------
import warnings
from typing import Any, Callable, Dict, List, Optional, Tuple, Union

import gym
import numpy as np

from stable_baselines3.common import type_aliases
from stable_baselines3.common.vec_env import DummyVecEnv, VecEnv, VecMonitor, is_vecenv_wrapped


def runModel():
    env.reset()
    next_la = [0, 0]
    rew_sum = 0
    for _ in range(200):

        obs, reward, done, truncated, info = env.step(next_la)
        if(env.vehicle.crashed or not env.vehicle.on_road):
            print("CRASHED")
            break
        rew_sum += reward
        next_la = learner.predict(obs)[0]

    return rew_sum


if train_model:
    print("evaluating policy before training")
    rewards1 = runModel()
    print("Rewards: ", rewards1)
    for iteration in range(0, 10):
        print("iteration "+str(iteration))
        print("training")
        gail_trainer.train(50000)
        rewards2 = runModel()
        print("Rewards: ", rewards2)
        print("saving policy")
        learner.save("gail-policy")
else:
    learner = MlpPolicy.load("gail-policy")

print("evaluating policy")
for i in range(0, 8):
    rewards2 = runModel()
    print("Rewards: ", rewards2)



