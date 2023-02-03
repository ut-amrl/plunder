import numpy as np
import gym
import highway_env
from stable_baselines3 import PPO
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.ppo import MlpPolicy
import math
import random

from imitation.algorithms.adversarial.gail import GAIL
from imitation.data import rollout
from imitation.data.wrappers import RolloutInfoWrapper
from imitation.rewards.reward_nets import BasicRewardNet
from imitation.util.networks import RunningNorm
from imitation.util.util import make_vec_env
from imitation.data.types import TrajectoryWithRew


rng = np.random.default_rng(0)

env = gym.make("highway-v0")


# expert, replaced by our policy
print("p0")
expert = PPO(policy=MlpPolicy, env=env, n_steps=64)

print("p1")
expert.learn(1)
print("expert finished learning")

rollouts = rollout.rollout(
    expert,
    make_vec_env(
        "highway-v0",
        n_envs=2,
        post_wrappers=[lambda env, _: RolloutInfoWrapper(env)],
        rng=rng,
    ),
    rollout.make_sample_until(min_timesteps=None, min_episodes=3),
    rng=rng,
)



venv = make_vec_env("highway-v0", n_envs=8, rng=rng)
print(venv.observation_space)
print(venv.action_space)



# TODO: identify data format for rollouts and venv

learner = PPO(env=venv, policy=MlpPolicy)
reward_net = BasicRewardNet(
    venv.observation_space,
    venv.action_space,
    normalize_input_layer=RunningNorm,
)
gail_trainer = GAIL(
    demonstrations=rollouts,
    demo_batch_size=32,
    gen_replay_buffer_capacity=2048,
    n_disc_updates_per_round=4,
    venv=venv,
    gen_algo=learner,
    reward_net=reward_net,
)

print("training")
gail_trainer.train(20000)
print("evaluating policy")
rewards, _ = evaluate_policy(learner, venv, 100, return_episode_rewards=True)
print("Rewards:", rewards)

