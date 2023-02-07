import gym
from gym.envs.registration import register
import numpy as np

register(
        id='highway-custom',
        entry_point='env_mod.envs:HighwayEnv',
    )

train_model = True



# ----------------------------------------------------------------------------------------
rng = np.random.default_rng(0)
config={}
config['simulation_frequency']=20
config['policy_frequency']=5 # Runs once every 4 simulation steps
config['lanes_count']=4
config['observation']={
    'type': 'CustomObservation',
    'vehicles_count': 10,
    'features': ['presence', 'x', 'y', 'vx', 'vy', 'heading'],
    'absolute': False
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
    reader = csv.reader(open("2D-highway-env/python-gen/data"+str(n)+".csv", "r"))
    next(reader)
    traj_obs = []
    traj_acts = []
    for line in reader:
        # LA
        traj_acts.append([line[20], line[21]])
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
        'obs': obs,
        'acts': acts,
        'next_obs': next_obs,
        'dones': dones
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
learner = PPO(env=venv, policy=MlpPolicy)         # PPO = SOTA RL, generator
# MLP = multi layer perceptron (simple NN)

reward_net = BasicRewardNet(
    venv.observation_space,
    venv.action_space,
    normalize_input_layer=RunningNorm,
)                                                 # discriminator
gail_trainer = GAIL(
    demonstrations=rollouts,                       # expert demos
    demo_batch_size=200,
    gen_replay_buffer_capacity=2048,
    n_disc_updates_per_round=4,
    venv=venv,                                     # environment
    gen_algo=learner,
    reward_net=reward_net,
)







# ----------------------------------------------------------------------------------------
import warnings
from typing import Any, Callable, Dict, List, Optional, Tuple, Union

import gym
import numpy as np

from stable_baselines3.common import type_aliases
from stable_baselines3.common.vec_env import DummyVecEnv, VecEnv, VecMonitor, is_vecenv_wrapped

if train_model:
    print("evaluating policy before training")
    rewards1, _ = evaluate_policy(learner, venv, 10, return_episode_rewards=True, deterministic=False)
    print("Rewards: ", rewards1)
    print("training")
    gail_trainer.train(20000)
    print("saving policy")
    learner.save("gail-policy")


print("evaluating policy")
learner = MlpPolicy.load("gail-policy")
rewards2, _ = evaluate_policy(learner, venv, 10, return_episode_rewards=True, deterministic=False)
print("Rewards: ", rewards2)

# actions, states = model.predict(observations, state=states, episode_start=episode_starts, deterministic=deterministic)
#         observations, rewards, dones, infos = env.step(actions)


