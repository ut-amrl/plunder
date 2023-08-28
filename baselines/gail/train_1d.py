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

train_model = True
rng = np.random.default_rng(3256)
dataPath = "data-1d"
n_timesteps = 125
n_train = 10
n_test = 20
env_id = "env-1d-v1"
policy_saved_name = "gail-1d-policy-v2"
train_steps = 2048*100
n_loop = 1000

configs = []

def motor_model_possibilities(decMax, accMax, prev_la):
    acc0 = min(prev_la+1, accMax)
    acc2 = max(prev_la-1, decMax)
    acc1 = acc0 if prev_la<=0 else acc2
    return [acc0, acc1, acc2]




def read_demo(n):
    reader = csv.reader(open(dataPath+"/data"+str(n)+".csv", "r"))
    next(reader)
    traj_obs = []
    traj_acts = []
    traj_ha = []
    prev_la = [0]
    for line in reader:
        traj_acts.append([float(line[1])])
        cur_obs = list(map(float, line[2:7]+prev_la))
        cur_obs += motor_model_possibilities(cur_obs[1], cur_obs[2], prev_la[0])
        traj_obs.append(cur_obs)
        traj_ha.append(int(line[8]))
        prev_la = [float(line[1])]
    return (traj_obs, traj_acts, traj_ha)



def get_expert_traj(n):
    obs_res, acts, _ = read_demo(n)
    obs = obs_res[:-1]
    acts = acts[:-1]
    next_obs = obs_res[1:]
    dones = [False]*(len(obs))
    return {
        'obs': np.array(obs, dtype=float),
        'acts': np.array(acts, dtype=float),
        'next_obs': np.array(next_obs, dtype=float),
        'dones': np.array(dones)
    }

def get_expert_trajs(n_demos):
    rollouts=[]
    for i in range(n_demos):
        rollouts.append(get_expert_traj(i))
    return rollouts

def setup_venv():
    venv = make_vec_env(env_id, n_envs=n_train, rng=rng)
    for n in range(n_train):
        reader = csv.reader(open(dataPath+"/data"+str(n)+".csv", "r"))
        next(reader)
        info = next(reader)
        venv.env_method("config", info[3], info[4], info[5], info[7], indices=[n])
    return venv


def print_trajs(env, learner):
    env.reset()
    next_la = [0]
    v_list = []
    x_list = []
    la_list = []
    for _ in range(n_timesteps):
        obs, _, _, _ = env.step(next_la)
        next_la = learner.predict(obs)[0]
        x_list.append(float('{0:.2f}'.format(obs[0])))
        v_list.append(float('{0:.2f}'.format(obs[4])))
        la_list.append(float('{0:.2f}'.format(next_la[0])))
    print(x_list)
    print(v_list)
    print(la_list)


def debugger(env):
    env.reset()
    obs, acts, _ = read_demo(3)
    env_obs = env.reset()
    env.config(obs[0][1], obs[0][2], obs[0][3], obs[0][5])
    for i in range(n_timesteps):
        print(env_obs)
        print(obs[i])
        print(acts[i])
        next_la = acts[i]
        env_obs, _, _, _ = env.step(next_la)


def compareModelWithGt(n, learner):
    # ll = 0
    obs, acts, ha = read_demo(n)
    obs = np.array(obs, dtype=float)
    acts = np.array(acts, dtype=float)
    ha = np.array(ha, dtype=int)
    action_vals = [obs[0][2], 0, obs[0][1]]
    model_acts = [learner.predict(o)[0][0] for o in obs]
    for i in range(len(acts)):
        print('{0:.2f}'.format(model_acts[i])+" "+'{0:.2f}'.format(acts[i][0])+" "+'{0:.2f}'.format(model_acts[i]-acts[i][0]))
        # ll += log(norm_pdf(gt_acts[i], stdev, model_acts[i]))
    # return ll/len(acts)
    return 0

def test_models(learner):
    ll_tot = 0.
    for i in range(0, n_train+n_test):
        print("\n\niter "+str(i))
        q = compareModelWithGt(i, learner)
        print(q)
        ll_tot += q
    print("average ll " + str(ll_tot / n_train))







register(id=env_id, entry_point='custom_envs.envs:Env_1d')
rollouts = get_expert_trajs(n_train)
venv = setup_venv()


learner = PPO(
    env=venv,
    policy=MlpPolicy,
    batch_size=1024,
    ent_coef=0.01,
    learning_rate=0.0004,
    n_epochs=3,
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



# env_obs = read_demo(3)[0][0]
test_env = gym.make(env_id)
# test_env.config(float(env_obs[1]), float(env_obs[2]), float(env_obs[3]), float(env_obs[5]))
# # print_trajs(test_env, learner)
debugger(test_env)
# debugger(test_env)

# exit()


if train_model:
    for i in range(n_loop):
        print("LOOP # "+str(i))
        gail_trainer.train(train_steps)
        test_models(learner)
        for i in range(0, n_train+n_test):
            print()
            print()
            print("iter "+str(i))
            env_obs = read_demo(i)[0][0]
            test_env.config(float(env_obs[1]), float(env_obs[2]), float(env_obs[3]), float(env_obs[5]))
            print_trajs(test_env, learner)
        learner.save(policy_saved_name)
else:
    learner = MlpPolicy.load(policy_saved_name)


test_models(learner)









