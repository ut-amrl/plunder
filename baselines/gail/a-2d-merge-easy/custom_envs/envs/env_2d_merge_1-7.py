import copy
import os
from typing import List, Tuple, Optional, Callable, TypeVar, Generic, Union, Dict, Text
import gym
from gym import Wrapper
from gym.wrappers import RecordVideo
from gym.utils import seeding
import gym.spaces as spaces
import numpy as np

from highway_env import utils
from highway_env.envs.common.action import action_factory, Action, DiscreteMetaAction, ActionType
from highway_env.envs.common.observation import observation_factory, ObservationType, KinematicObservation
from highway_env.envs.common.finite_mdp import finite_mdp
from highway_env.envs.common.graphics import EnvViewer
from highway_env.vehicle.behavior import IDMVehicle, LinearVehicle
from highway_env.vehicle.controller import MDPVehicle
from highway_env.vehicle.kinematics import Vehicle

np.set_printoptions(suppress=True)

######## Configuration ########
lane_diff = 4 # Distance lanes are apart from each other
use_absolute_lanes = True
KinematicObservation.normalize_obs = lambda self, df: df






class CustomObservation(KinematicObservation):

    def __init__(self, env: 'AbstractEnv',
                 features: List[str] = None,
                 vehicles_count: int = 5,
                 features_range: Dict[str, List[float]] = None,
                 absolute: bool = False,
                 order: str = "sorted",
                 normalize: bool = True,
                 clip: bool = True,
                 see_behind: bool = False,
                 observe_intentions: bool = False,
                 **kwargs: dict) -> None:

        super().__init__(env, features, vehicles_count, features_range, absolute, order, normalize, clip, see_behind, observe_intentions, **kwargs)


    def space(self) -> spaces.Space:
        return spaces.Box(low=-np.inf, high=np.inf, shape=(20,), dtype=np.float64)


    # Observation preprocessing ---------------------------------------------------
    def laneFinder(self, y):
        return round(y / lane_diff)

    def classifyLane(self, obs):
        lane_class = []
        for vehicle in obs:
            lane_class.append(self.laneFinder(vehicle[2]))
        return lane_class
        
    def closestInLane(self, obs, lane, lane_class):
        for i in range(0, len(obs)):
            if obs[i][0] == 0: # not present
                continue
            if lane_class[i] == lane: # in desired lane
                return obs[i]
        
        return [0, 1000000000, lane * lane_diff, 0, 0, 0] # No car found

    def closestVehicles(self, obs, lane_class):
        closestLeft = self.closestInLane(obs[1:], -1, lane_class[1:])
        closestFront = self.closestInLane(obs[1:], 0, lane_class[1:])
        closestRight = self.closestInLane(obs[1:], 1, lane_class[1:])

        # Handle edges (in rightmost or leftmost lane)
        if lane_class[0] == 0: # In leftmost lane: pretend there is a vehicle to the left
            closestLeft = obs[0].copy()
            closestLeft[1] = 0
            closestLeft[2] = -lane_diff
        if lane_class[0] == self.env.config['lanes_count'] - 1: # In rightmost lane: pretend there is a vehicle to the right
            closestRight = obs[0].copy()
            closestRight[1] = 0
            closestRight[2] = lane_diff
        
        return (closestLeft, closestFront, closestRight)
    # ---------------------------------------------------------------------




    def observe(self) -> np.ndarray:
        obs = super().observe()
        lane_class = self.classifyLane(obs)
        closest = self.closestVehicles(obs, lane_class)
        flat_obs = []

        for prop in obs[0][1:]:
            flat_obs.append(round(prop, 3))
        
        for v in closest:
            for prop in v[1:]:
                flat_obs.append(round(prop, 3))

        return np.array(flat_obs)



def observation_factory2(env: 'AbstractEnv', config: dict) -> ObservationType:
    try:
        return observation_factory(env, config)
    except ValueError:
        if config["type"] == "CustomObservation":
            return CustomObservation(env, **config)
        else:
            raise ValueError("Unknown observation type")

  
from typing import Dict, Text
import numpy as np
from highway_env import utils
from highway_env.envs.common.action import Action
from highway_env.road.road import Road, RoadNetwork
from highway_env.utils import near_split
from highway_env.vehicle.controller import ControlledVehicle
from highway_env.vehicle.kinematics import Vehicle
from highway_env.envs.common.abstract import AbstractEnv

Observation = np.ndarray


class HighwayEnv(AbstractEnv):
    """
    A highway driving environment.
    The vehicle is driving on a straight highway with several lanes, and is rewarded for reaching a high speed,
    staying on the rightmost lanes and avoiding collisions.
    """

    def define_spaces(self) -> None:
        self.observation_type = observation_factory2(self, self.config["observation"])
        self.action_type = action_factory(self, self.config["action"])
        self.observation_space = self.observation_type.space()
        self.action_space = self.action_type.space()

    @classmethod
    def default_config(cls) -> dict:
        config = super().default_config()
        config.update({
            "observation": {
                "type": "Kinematics"
            },
            "action": {
                "type": "DiscreteMetaAction",
            },
            "lanes_count": 4,
            "vehicles_count": 50,
            "controlled_vehicles": 1,
            "initial_lane_id": None,
            "duration": 40,  # [s]
            "ego_spacing": 2,
            "vehicles_density": 1,
            "collision_reward": -1,    # The reward received when colliding with a vehicle.
            "right_lane_reward": 0.1,  # The reward received when driving on the right-most lanes, linearly mapped to
                                       # zero for other lanes.
            "high_speed_reward": 0.4,  # The reward received when driving at full speed, linearly mapped to zero for
                                       # lower speeds according to config["reward_speed_range"].
            "lane_change_reward": 0,   # The reward received at each lane change action.
            "reward_speed_range": [20, 30],
            "normalize_reward": True,
            "offroad_terminal": False
        })
        return config

    def _reset(self) -> None:
        self._create_road()
        self._create_vehicles()



    def _create_road(self) -> None:
        """Create a road composed of straight adjacent lanes."""
        self.road = Road(network=RoadNetwork.straight_road_network(self.config["lanes_count"], speed_limit=30),
                         np_random=self.np_random, record_history=self.config["show_trajectories"])

    def _create_vehicles(self) -> None:
        """Create some new random vehicles of a given type, and add them on the road."""
        other_vehicles_type = utils.class_from_path(self.config["other_vehicles_type"])
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
                vehicle = other_vehicles_type.create_random(self.road, spacing=1 / self.config["vehicles_density"])
                vehicle.randomize_behavior()
                self.road.vehicles.append(vehicle)

    def _reward(self, action: Action) -> float:
        """
        The reward is defined to foster driving at high speed, on the rightmost lanes, and to avoid collisions.
        :param action: the last action performed
        :return: the corresponding reward
        """
        rewards = self._rewards(action)
        reward = sum(self.config.get(name, 0) * reward for name, reward in rewards.items())
        if self.config["normalize_reward"]:
            reward = utils.lmap(reward,
                                [self.config["collision_reward"],
                                 self.config["high_speed_reward"] + self.config["right_lane_reward"]],
                                [0, 1])
        reward *= rewards['on_road_reward']
        return reward

    def _rewards(self, action: Action) -> Dict[Text, float]:
        neighbours = self.road.network.all_side_lanes(self.vehicle.lane_index)
        lane = self.vehicle.target_lane_index[2] if isinstance(self.vehicle, ControlledVehicle) \
            else self.vehicle.lane_index[2]
        # Use forward speed rather than speed, see https://github.com/eleurent/highway-env/issues/268
        forward_speed = self.vehicle.speed * np.cos(self.vehicle.heading)
        scaled_speed = utils.lmap(forward_speed, self.config["reward_speed_range"], [0, 1])
        return {
            "collision_reward": float(self.vehicle.crashed),
            "right_lane_reward": lane / max(len(neighbours) - 1, 1),
            "high_speed_reward": np.clip(scaled_speed, 0, 1),
            "on_road_reward": float(self.vehicle.on_road)
        }

    def _is_terminated(self) -> bool:
        return (self.vehicle.crashed or not self.vehicle.on_road) or (self.time >= self.config["duration"])

    def _is_truncated(self) -> bool:
        return False