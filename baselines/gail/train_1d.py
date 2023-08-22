from gym.envs.registration import register
import gym
import csv
import numpy as np
from imitation.algorithms.adversarial.gail import GAIL
from stable_baselines3.ppo import MlpPolicy
from stable_baselines3 import PPO
from imitation.util.util import make_vec_env
from imitation.rewards.reward_nets import BasicRewardNet
from imitation.util.networks import RunningNorm


rng = np.random.default_rng(0)
dataPath = "data-1d"
n_timesteps = 126
register(id='env-1d-v1', entry_point='custom_envs.envs:Env_1d')
env_base = gym.make("env-1d-v1")
env_base.config(-5., 6., 12., 100.)
def runSim(env, iter):
    env.reset()
    for _ in range(n_timesteps):
        obs, rew, done, truncated, info = env.step(1)
        print(obs)
        if done:
          break




def read_demo(n):
    reader = csv.reader(open(dataPath+"/data"+str(n)+".csv", "r"))
    next(reader)
    traj_obs = []
    traj_acts = []
    traj_ha = []
    for line in reader:
        traj_acts.append([line[1]])
        traj_obs.append(line[2:7])
        traj_ha.append(line[8])
    return (traj_obs, traj_acts, traj_ha)

def gen_traj(n):
    obs, acts, _ = read_demo(n)
    next_obs = obs[1:]
    next_obs.append(next_obs[-1])
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

rollouts = gen_trajs(8)
venv = make_vec_env("env-1d-v1", n_envs=8, rng=rng)

def setup_venv():
    for n in range(8):
        reader = csv.reader(open(dataPath+"/data"+str(n)+".csv", "r"))
        next(reader)
        info = next(reader)
        venv.env_method("config", info[3], info[4], info[5], info[7], indices=[n])
setup_venv()

learner = PPO(
    env=venv,
    policy=MlpPolicy,
    batch_size=64,
    ent_coef=0.0,
    learning_rate=0.0002,
    n_epochs=50,
)

reward_net = BasicRewardNet(
    venv.observation_space,
    venv.action_space,
    normalize_input_layer=RunningNorm,
)                                                  # discriminator

gail_trainer = GAIL(
    demonstrations=rollouts,                       # expert demos
    demo_batch_size=n_timesteps,
    gen_replay_buffer_capacity=n_timesteps*2,
    n_disc_updates_per_round=3,
    venv=venv,                                     # environment
    gen_algo=learner,
    reward_net=reward_net,
)


gail_trainer.train(20000)

def runModel(env, iter):
    env.reset()
    next_la = [0]
    v_list = []
    x_list = []
    la_list = []
    for _ in range(n_timesteps):
        obs, rew, _, info = env.step(next_la)
        next_la = learner.predict(obs)[0]
        x_list.append(obs[0])
        v_list.append(obs[4])
        la_list.append(next_la[0])
    print(x_list)
    print(v_list)
    print(la_list)


for i in range(0, 8):
    print()
    print()
    print("iter "+str(i))
    runModel(env_base, i)




def compareModelWithGt(n):
    ll = 0
    obs, acts, ha = read_demo(n)
    obs = np.array(obs, dtype=float)
    acts = np.array(acts, dtype=float)
    ha = np.array(ha, dtype=int)
    action_vals = [obs[0][2], 0, obs[0][1]]
    gt_acts = [action_vals[a] for a in ha]
    model_acts = [learner.predict(o)[0][0] for o in obs]
    for i in range(len(acts)):
        print(str(model_acts[i])+" "+str(gt_acts[i])+" "+str(model_acts[i]-gt_acts[i]))
        # ll += log(norm_pdf(gt_acts[i], stdev, model_acts[i]))
    # return ll/len(acts)
    return 0


n_samples = 8
ll_tot = 0.
for i in range(0, n_samples):
    print()
    print()
    print("iter "+str(i))
    q = compareModelWithGt(i)
    print(q)
    ll_tot += q
print("average ll " + str(ll_tot / n_samples))



