from gym.envs.registration import register
import pandas as pd
from imitation.util.util import make_vec_env
import numpy as np
import warnings
warnings.filterwarnings("ignore")


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
_n_traj_tot = 1




# globals -----------------------------------------------------------------
_traj_all = None
_venv = None



# custom configurations --------------------------------------------------------
_config1 = {
  "simulation_frequency": 24,
  "policy_frequency": 8,
  "lanes_count": 6,
  "initial_lane_id": 0,
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
  "observation": {
    'type': 'Kinematics',
    'vehicles_count': 50,
    'features': ['presence', 'x', 'y', 'vx', 'vy', 'heading'],
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




# get expert data -----------------------------------------------------------------
def get_single_expert_traj(n):
  data = pd.read_csv(_data_path+f"/data{n}.csv", skipinitialspace=True)
  data = data[_ha_column + _la_column + _feature_column]
  return data

def get_all_expert_traj():
  return [get_single_expert_traj(i) for i in range(_n_traj_tot)]

register(id=_env_id, entry_point=_custom_env_class_path)
_traj_all = get_all_expert_traj()




# configure environments --------------------------------------------------
_venv = make_vec_env(_env_id, n_envs=_n_traj_tot, rng=_rng)
for i in range(_n_traj_tot):
  _venv.env_method("set_custom_config", get_config_of_env_n(i), _feature_indices, indices=[i])
  res = _venv.env_method("reset", indices=[i])
  print(res[0][0])
  for _ in range(9):
    res = _venv.env_method("step", [0,0], indices=[i])
    print(res[0][0])
  print("-------------")
  print(_traj_all[i][:10])




