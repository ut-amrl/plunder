from gym.envs.registration import register
import pandas as pd
from imitation.util.util import make_vec_env
import numpy as np
import warnings
warnings.filterwarnings("ignore")
from imitation.algorithms.adversarial.gail import GAIL
from stable_baselines3.ppo import MlpPolicy
from stable_baselines3 import PPO
from imitation.rewards.reward_nets import BasicRewardNet
from imitation.util.networks import RunningNorm
import gym


# constants -------------------------------------------------------
_rng = np.random.default_rng()
_custom_env_class_path = 'custom_envs.envs:Env_2d_merge'
_env_id = "env-2d-merge-v0"
_data_path = "data"
_ha_column = ["HA"]
_la_column = ["LA.steer", "LA.acc"]
_feature_column = ["x", "vx", "l_x", "f_x", "r_x"]
_feature_indices = [0, 2, 5, 10, 15]
_n_traj_train = 10
_n_traj_test = 20
_n_traj_tot = 30
_n_timesteps = 74



# custom configurations --------------------------------------------------------
_config1 = {
  "simulation_frequency": 24,
  "policy_frequency": 8,
  "lanes_count": 6,
  "initial_lane_id": 0,
  'vehicles_count': 50,
  "observation": {
    'type': 'Kinematics',
    'vehicles_count': 10,
    'features': ['presence', 'x', 'y', 'vx', 'vy', 'heading'],
    'absolute': True
  },
  "action": {
    "type": "ContinuousAction"
  }
}
_config2 = {
  "simulation_frequency": 24,
  "policy_frequency": 8,
  "lanes_count": 8,
  "initial_lane_id": 0,
  'vehicles_count': 50,
  "observation": {
    'type': 'Kinematics',
    'features': ['presence', 'x', 'y', 'vx', 'vy', 'heading'],
    'vehicles_count': 50,
    'absolute': True
  },
  "action": {
    "type": "ContinuousAction"
  }
}
def get_config_of_env_n(n):
  if n < 25:
    return _config1
  else:
    return _config2




# LA option calculations --------------------------------------------------------
TURN_HEADING = 0.15 # Target heading when turning
TURN_TARGET = 30 # How much to adjust when targeting a lane (higher = smoother)
MAX_VELOCITY = 45 # Maximum velocity
LA_COUNT = 4
lane_diff = 4
steer_max = .3
acc_max = 30
def laneFinder(y):
    return round(y / lane_diff)
def motor_model(ha, data, last_la):
    last_la = [last_la[0] * steer_max, last_la[1] * acc_max]
    target_acc = 0.0
    target_heading = 0.0

    if ha == 0:
        target_acc = MAX_VELOCITY - data[1]

        target_y = laneFinder(data[1]) * 4
        target_heading = np.arctan((target_y - data[1]) / TURN_TARGET)
    elif ha == 1:
        target_acc = data[12] - data[2]

        target_y = laneFinder(data[1]) * 4
        target_heading = np.arctan((target_y - data[1]) / TURN_TARGET)
    elif ha == 2:
        target_acc = -0.5
        target_heading = -TURN_HEADING
    else:
        target_acc = -0.5
        target_heading = TURN_HEADING

    target_steer = target_heading - data[4]
    if target_steer > last_la[0]:
        target_steer = min(target_steer, last_la[0] + 0.08)
    else:
        target_steer = max(target_steer, last_la[0] - 0.08)

    if target_acc > last_la[1]:
        target_acc = min(target_acc, last_la[1] + 4)
    else:
        target_acc = max(target_acc, last_la[1] - 6)
    res = [target_steer / steer_max, target_acc / acc_max]
    res[0] = min(max(res[0], -1), 1)
    res[1] = min(max(res[1], -1), 1)
    return res
def motor_model_options(proc_obs, last_la):
    la_options = []
    for ha in range(LA_COUNT):
        la_options.extend(motor_model(ha, proc_obs, last_la))
    return la_options
def motor_model_options_all(proc_obs_arr, last_la_arr):
  return np.array([motor_model_options(obs, la) for obs, la in zip(proc_obs_arr, last_la_arr)])




# get expert data -----------------------------------------------------------------
def get_single_expert_df(n):
  return pd.read_csv(_data_path+f"/data{n}.csv", skipinitialspace=True)
def get_single_expert_traj(n):
  data = get_single_expert_df(n)
  ha = data[_ha_column].to_numpy()
  la = data[_la_column].to_numpy()
  la = np.divide(la, [steer_max, acc_max])
  features = data[_feature_column].to_numpy()
  everything = data.to_numpy()
  la = np.append(np.array([[0,0]]), la, axis=0)

  obs = np.append(features[:-1], la[:-2], axis=1)
  next_obs = np.append(features[1:], la[1:-1], axis=1)
  acts = la[1:-1]

  obs = np.append(obs, motor_model_options_all(everything[:-1], la[:-2]), axis=1)
  next_obs = np.append(next_obs, motor_model_options_all(everything[1:], la[1:-1]), axis=1)

  obs = obs[:_n_timesteps]
  next_obs = next_obs[:_n_timesteps]
  acts = acts[:_n_timesteps]
  dones = np.array([False]*_n_timesteps)

  return {
    "obs": obs,
    "next_obs": next_obs,
    "acts": acts,
    "dones": dones
  }
def get_all_expert_traj():
  return [get_single_expert_traj(i) for i in range(_n_traj_tot)]

_traj_all = get_all_expert_traj()
_traj_train = _traj_all[:_n_traj_train]




# configure environments --------------------------------------------------
register(id=_env_id, entry_point=_custom_env_class_path)
_venv = make_vec_env(_env_id, n_envs=_n_traj_train, rng=_rng)
for i in range(_n_traj_train):
  _venv.env_method("set_custom_config", get_config_of_env_n(i), _feature_indices, _n_timesteps, indices=[i])
_env_test = gym.make(_env_id)
_env_test.set_custom_config(get_config_of_env_n(0), _feature_indices, _n_timesteps)




# evaluation ---------------------------------------------------------
def evaluate(model, trajectories):
  for i, traj in enumerate(trajectories):
    print(f"DATA ENV {i}")
    for features, action in zip(traj["obs"], traj["acts"]):
      predicted_action = list(model.predict(features)[0])
      print(f"steer {predicted_action[0]*steer_max:.3f} --- {action[0]*steer_max:.3f}  \
            --- acc {predicted_action[1]*acc_max:.3f} --- {action[1]*acc_max:.3f}")

def sanity(model):
  print("SANITY")
  prev_obs = _env_test.reset()
  print([f"{num:.3f}" for num in prev_obs])
  for i in range(0,_n_timesteps):
    predicted_action = model.predict(prev_obs)[0]
    prev_obs = _env_test.step(predicted_action)[0]
    print([f"{num:.3f}" for num in prev_obs])
    if (_env_test.vehicle.crashed or not _env_test.vehicle.on_road):
      print("CRASHED")
      break

def debug():
  traj = _traj_all[0]
  res = [_venv.env_method("reset", indices=[0])]
  for i in range(0,_n_timesteps):
    print("data: "+str([f"{num:.3f}" for num in traj["obs"][i]]))
    print("siml: "+str([f"{num:.3f}" for num in res[0][0]]))
    res = _venv.env_method("step", traj["acts"][i], indices=[0])
  print("RESET")
  res = [_venv.env_method("reset", indices=[0])]
  for i in range(0,_n_timesteps):
    print("data: "+str([f"{num:.3f}" for num in traj["obs"][i]]))
    print("siml: "+str([f"{num:.3f}" for num in res[0][0]]))
    res = _venv.env_method("step", traj["acts"][i], indices=[0])

debug()

# GAIL --------------------------------------------------------------
_n_train_loops = 100000
_n_train_steps = 15
_learner = PPO(
    env=_venv,
    policy=MlpPolicy,
    batch_size=_n_timesteps,
    ent_coef=0.0002,
    learning_rate=0.0005,
    n_epochs=10,
    gamma=.99,
    n_steps=_n_timesteps
)
_reward_net = BasicRewardNet(
    _venv.observation_space,
    _venv.action_space,
    normalize_input_layer=RunningNorm,
)
_gail_trainer = GAIL(
    demonstrations=_traj_train,
    demo_batch_size=_n_timesteps,                 # cons
    gen_replay_buffer_capacity=_n_timesteps*2,    # cons
    gen_train_timesteps=15,                       # gen per round
    n_disc_updates_per_round=3,                   # disc per round
    venv=_venv,
    gen_algo=_learner,
    reward_net=_reward_net,
)

for i in range(_n_train_loops):
    print("LOOP # "+str(i))
    _gail_trainer.train(_n_train_steps)           # total rounds = _n_train_steps / gen_train_timesteps
    evaluate(_learner, _traj_all)
    sanity(_learner)


