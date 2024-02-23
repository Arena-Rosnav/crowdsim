import dataclasses
import enum
import typing

import numpy as np

import pedsim_msgs.msg as pedsim_msgs
import crowdsim_msgs.msg as crowdsim_msgs
import std_msgs.msg as std_msgs
import geometry_msgs.msg as geometry_msgs

T = typing.TypeVar("T")

def NList(l: typing.Optional[typing.List[T]]) -> typing.List[T]:
    return [] if l is None else l

def msg_to_vec(msg: typing.Union[geometry_msgs.Point, geometry_msgs.Vector3]) -> np.ndarray:
    return np.array([msg.x, msg.y, msg.z])

class Dist:
    def apply(self, modifier: typing.Optional[float] = None) -> "Dist": raise NotImplementedError()
    def generate(self, modifier: typing.Optional[float] = None, rng: typing.Optional[np.random.Generator] = None) -> float: raise NotImplementedError()

class ConstantDist(Dist):
    _value: float
    def __init__(self, value: float): self._value = value
    def apply(self, modifier): return self
    def generate(self, modifier, rng): return self._value

class NormalDist(Dist):

    _min: typing.Optional[float]
    _max: typing.Optional[float]

    _mean: float
    _stdev: float

    def __init__(self, mean: float, stdev: float, min_: typing.Optional[float] = None, max_: typing.Optional[float] = None):
        self._min = min_
        self._max = max_

        self._mean = mean
        self._stdev = stdev

    def apply(self, modifier: typing.Optional[float]):
        if modifier is None: modifier = 0
        return NormalDist(
            mean = self._mean + self._stdev * modifier,
            stdev = self._stdev,
            min_ = self._min,
            max_ = self._max
        )

    def generate(self, modifier: typing.Optional[float], rng: typing.Optional[np.random.Generator]) -> float:
        if rng is None: rng = np.random.default_rng()
        if modifier is None: modifier = 0

        value = (rng.standard_normal() + modifier) * self._stdev + self._mean
        if self._min is not None: value = max(self._min, value)
        if self._max is not None: value = min(self._max, value)
        return value

class RandomConfig(typing.Dict[str, Dist]):
    def apply(self, entries: typing.Optional[typing.Dict[str, typing.Union[float, Dist]]] = None) -> "RandomConfig":
        if entries is None: entries = dict()
        return RandomConfig({k:v if isinstance(v, Dist) else v.apply(entries.get(k,0)) for k,v in self.items()})

    def generate(self, entries: typing.Optional[typing.Dict[str, float]] = None, rng: typing.Optional[np.random.Generator] = None) -> typing.Dict[str, float]:
        if entries is None: entries = dict()
        if rng is None: rng = np.random.default_rng()
        return {k:v.generate(entries.get(k,0), rng) for k,v in self.items()}

# INPUT

InMsg = pedsim_msgs.PedsimAgentsDataframe

@dataclasses.dataclass
class InData:
    header: std_msgs.Header
    agents: typing.List[pedsim_msgs.AgentState]
    robots: typing.List[pedsim_msgs.RobotState]
    groups: typing.List[pedsim_msgs.AgentGroup]
    waypoints: typing.List[pedsim_msgs.Waypoint]
    walls: typing.List[pedsim_msgs.Wall]
    obstacles: typing.List[pedsim_msgs.Obstacle]

@dataclasses.dataclass
class OutDatum:
    id: np.string_
    force: np.ndarray
    social_state: np.string_

OutMsg = pedsim_msgs.AgentFeedbacks
    
class WorkData:

    @classmethod
    def construct(cls, in_data: InData) -> "WorkData":
        data = WorkData(
            n_agents=len(in_data.agents)
        )
        
        for i, agent in enumerate(in_data.agents):
            data.id.append(agent.id)
            data.force[i] = agent.forces.force.x, agent.forces.force.y, agent.forces.force.z
            data.social_state.append(agent.social_state)
            data.vmax[i] = 1.

        return data

    # _storage: np.ndarray

    id: typing.List
    force: np.ndarray
    social_state: typing.List
    vmax: np.ndarray

    def __init__(self, n_agents: int):
        self.id = list()
        self.social_state = list()

        self._storage = np.zeros((n_agents, 4))
        self.force = self._storage[:,0:3]
        self.vmax = self._storage[:,3]

    def msg(self, header: typing.Optional[std_msgs.Header] = None) -> OutMsg:

        return OutMsg(**dict(
            agents = [
                pedsim_msgs.AgentFeedback(
                    id = id,
                    force = geometry_msgs.Vector3(*force),
                    social_state = social_state,
                    vmax = vmax
                )
                for id, force, social_state, vmax
                in zip(self.id, self.force.tolist(), self.social_state, self.vmax.flat)
            ],
            **(dict(header=header) if header is not None else dict())
        ))



# SEMANTIC 

@enum.unique
class SemanticAttribute(enum.Enum):
    IS_PEDESTRIAN = "pedestrian"
    IS_PEDESTRIAN_MOVING = "pedestrian_moving"
    PEDESTRIAN_VEL_X = "pedestrian_vel_x"
    PEDESTRIAN_VEL_Y = "pedestrian_vel_y"
    PEDESTRIAN_TYPE = "pedestrian_type"

    SOCIAL_STATE = "social_state"

    STATE_STRESS = "stress"
    STATE_ENERGY = "energy"
    STATE_SOCIAL = "social"

SemanticData = typing.Dict[SemanticAttribute, typing.List[typing.Tuple[geometry_msgs.Point, float]]]
SemanticMsg = crowdsim_msgs.SemanticData

