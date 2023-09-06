from highway_env.envs.highway_env import HighwayEnv
from highway_env.envs import MDPVehicle, ControlledVehicle, Vehicle, highway_env
from highway_env.envs.common.observation import KinematicObservation
from highway_env.utils import near_split, class_from_path
import numpy as np
from highway_env.road.road import Road, RoadNetwork
from gym.utils import seeding


# copied from merge.py --------------------------------------------------------
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


# custom env ---------------------------------------------------------------
class Env_2d_merge(HighwayEnv):
  obs_indices = []
  custom_config_set = False
  def __init__(self): 
    super().__init__()
    self.np_random = None
    self.seed()
  def set_custom_config(self, custom_config, obs_indices):
    self.config.update(custom_config)
    self.obs_indices = obs_indices
    self.custom_config_set = True

  def raw_to_proc_obs(self, raw_obs):
    lane_class = classifyLane(raw_obs)
    closest = closestVehicles(self, raw_obs, lane_class)
    proc_obs = raw_obs[0][1:]
    for v in closest:
      proc_obs = np.append(proc_obs, v[1:])
    proc_obs = [proc_obs[i] for i in self.obs_indices]
    return proc_obs
  def process_output(self, raw_out):
    if self.custom_config_set:
      raw_obs = raw_out[0]
      proc_obs = self.raw_to_proc_obs(raw_obs[1:])
      return tuple([proc_obs, 0, False, {}])
    else:
      return raw_out
  def step(self, action):
    raw_out = super().step(action)
    return self.process_output(raw_out)
  def reset(self):
    raw_out = super().reset()
    return self.process_output(raw_out)
  def _create_vehicles(self):
    _create_vehicles_custom(self)
  def seed(self, seed=None):
    self.np_random, seed = seeding.np_random(seed)
    return [seed]
      
