from highway_env.envs.highway_env import HighwayEnv
from highway_env.envs import MDPVehicle, ControlledVehicle, Vehicle, highway_env
from highway_env.envs.common.observation import KinematicObservation
from highway_env.utils import near_split, class_from_path
import numpy as np
from highway_env.road.road import Road, RoadNetwork
from gym.utils import seeding
from gym.spaces import Box
from highway_env.envs.common.action import ContinuousAction


# copied from merge.py --------------------------------------------------------
steer_max = .3
acc_max = 30
lane_diff = 4
KinematicObservation.normalize_obs = lambda self, df: df # Don't normalize values
def _create_vehicles_custom(self) -> None:
        """Create some new random vehicles of a given type, and add them on the road."""
        other_vehicles_type = class_from_path(self.config["other_vehicles_type"])
        other_per_controlled = near_split(self.config["vehicles_count"], num_bins=self.config["controlled_vehicles"])

        self.controlled_vehicles = []
        for others in other_per_controlled:
            vehicle = Vehicle.create_random(
                self.road,
                speed=25,
                lane_id=self.config["initial_lane_id"],
                spacing=self.config["ego_spacing"]
            )
            vehicle = self.action_type.vehicle_class(self.road, vehicle.position, vehicle.heading, vehicle.speed)
            self.controlled_vehicles.append(vehicle)
            self.road.vehicles.append(vehicle)

            for _ in range(others):
                vehicle = other_vehicles_type.create_random(self.road, spacing=1.0 / self.config["vehicles_density"])
                vehicle.randomize_behavior()
                self.road.vehicles.append(vehicle)               
def laneFinder(y):
    return round(y / lane_diff)
def classifyLane(obs):
    lane_class = []
    for vehicle in obs:
        lane_class.append(laneFinder(vehicle[2]))
    return lane_class
def closestInLane(obs, lane, lane_class, ego):
    for i in range(len(obs)):
        if obs[i][0] == 0: # not present
            continue
        if lane_class[i] == lane: # in desired lane
            return obs[i]
    
    return [0, ego[1] + 100, lane * lane_diff, ego[3], ego[4], ego[5]] # No car found
def closestVehicles(env, obs, lane_class):
    ego_lane = laneFinder(obs[0][2])

    closestLeft = closestInLane(obs[1:], ego_lane - 1, lane_class[1:], obs[0])
    closestFront = closestInLane(obs[1:], ego_lane, lane_class[1:], obs[0])
    closestRight = closestInLane(obs[1:], ego_lane + 1, lane_class[1:], obs[0])

    # Handle edges (in rightmost or leftmost lane)
    if lane_class[0] == 0: # In leftmost lane: pretend there is a vehicle to the left
        closestLeft = obs[0].copy()
        closestLeft[2] = obs[0][2] - lane_diff
    if lane_class[0] == env.config['lanes_count'] - 1: # In rightmost lane: pretend there is a vehicle to the right
        closestRight = obs[0].copy()
        closestRight[2] = obs[0][2] + lane_diff
    
    return (closestLeft, closestFront, closestRight)
def special_act(self, action=None):
  self.controlled_vehicle.act({
      "steering": action[0] * steer_max,
      "acceleration": action[1] * acc_max
  })
ContinuousAction.act = special_act




# LA option calculations --------------------------------------------------------
TURN_HEADING = 0.15 # Target heading when turning
TURN_TARGET = 30 # How much to adjust when targeting a lane (higher = smoother)
MAX_VELOCITY = 45 # Maximum velocity
LA_COUNT = 4
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


# custom env ---------------------------------------------------------------
class Env_2d_merge(HighwayEnv):
  obs_indices = []
  custom_config_set = False
  last_la = [0,0]
  max_t = -1
  t = -1
  def __init__(self): 
    super().__init__()
    self.np_random = None
    self.seed()
    temp1 = [-1, -1]
    temp2 = [1, 1]
    self.observation_space = Box(low=np.array([0, 0, 0, 0, 0]+temp1*5), 
                                  high=np.array([600, 60, 600, 600, 600]+temp2*5), 
                                  shape=(15,), dtype=np.float32)
    self.action_space = Box(low=np.array(temp1), high=np.array(temp2), shape=(2,), dtype=np.float32)
  def set_custom_config(self, custom_config, obs_indices, max_t):
    self.config.update(custom_config)
    self.obs_indices = obs_indices
    self.custom_config_set = True
    self.max_t = max_t

  def raw_to_proc_obs(self, raw_obs):
    lane_class = classifyLane(raw_obs)
    closest = closestVehicles(self, raw_obs, lane_class)
    proc_obs = raw_obs[0][1:]
    for v in closest:
      proc_obs = np.append(proc_obs, v[1:])
    proc_obs_pruned = [proc_obs[i] for i in self.obs_indices]
    proc_obs_pruned.extend(self.last_la)
    proc_obs_pruned.extend(motor_model_options(proc_obs, self.last_la))
    return proc_obs_pruned
  def process_output(self, raw_out):
    raw_obs = raw_out[0]
    proc_obs = self.raw_to_proc_obs(raw_obs)
    terminated = self.t > self.max_t
    return (proc_obs, 0, terminated, {})
  def step(self, action):
    raw_out = super().step(action)
    self.last_la = action
    self.t += 1
    return self.process_output(raw_out)
  def _reset(self):
    super()._reset()
    self.last_la = [0,0]
    self.t = 0
  def reset(self):
    if self.custom_config_set:
      raw_out = super().reset()
      return self.process_output(raw_out)[0]
    else:
      return None
  def _create_vehicles(self):
    _create_vehicles_custom(self)
  def seed(self, seed=None):
    self.np_random, seed = seeding.np_random(seed)
    return [seed]
      
