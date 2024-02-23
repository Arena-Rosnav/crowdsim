import sismic.model
import sismic.io
import sismic.interpreter

import dynamic_reconfigure.client

import typing
import enum

import numpy as np
import numpy.typing as npt
from crowdsim_agents import utils
from crowdsim_agents.utils import InData, WorkData

import rospy

class Range(typing.NamedTuple):
    MIN: float = 0
    MAX: float = 100
    DEFAULT: float = 50

    @classmethod
    def parse(cls, *args: float) -> "Range":
        if len(args) == 0: return cls() 
        if len(args) == 1: return cls(DEFAULT=args[0])
        if len(args) == 2: return cls(MIN=args[0], MAX=args[1], DEFAULT=(args[1]-args[0])/2)
        return cls(MIN=args[0], MAX=args[1], DEFAULT=args[2]) 


class StatechartProvider:
    @classmethod
    def load(cls, filepath: str):
        def wrapper(inner: typing.Type[Agent]):
            class Wrapped(inner):
                __statechart: sismic.model.Statechart

                def __init__(self, *args, **kwargs):
                    super().__init__(*args, **kwargs)
                    self.__statechart = sismic.io.import_from_yaml(filepath=filepath)

                @property
                def statechart(self):
                    return self.__statechart
            
            return Wrapped
        return wrapper
    
    @property
    def statechart(self) -> sismic.model.Statechart:
        raise NotImplementedError(f"{type(self).__name__} has no associated statechart")


class Agent(StatechartProvider):

    MAGIC_NUMBER = 0x01

    _config: typing.Dict
    _runtime: typing.Dict
    _state: typing.Dict
    _animation: str
    _social_state: int
    _destination: typing.Optional[typing.Tuple[float, float, float]]

    #TODO move vmax randomization here (from pedsim_engine)
    _random_config = utils.RandomConfig()

    def __init__(self, id: str):

        self._config = dict()

        self._runtime = dict(
            id = id,
            f = float(rospy.get_param("pedsim_simulator/update_rate")),
            dt = 1 / float(rospy.get_param("pedsim_simulator/update_rate")),
            rng = np.random.default_rng(None)
        )

        self._state = dict()
        
        #dynamic_reconfigure.client.Client("")

        self._destination = None
        self._animation = ""

    def setup(self, config: typing.Optional[typing.Dict[str, float]] = None, context: typing.Optional[typing.Dict] = None) -> "Agent":
        if config is None: config = dict()
        if context is None: context = dict()

        self._config = self._random_config.generate(config, self._runtime.get("rng"))

        self._statemachine = sismic.interpreter.Interpreter(
            self.statechart,
            initial_context=dict(
                config = self._config,
                runtime = self._runtime,
                state = self._state,
                **context
            )
        )
        self._statemachine.bind(self.event_handler)
        return self

    def event_handler(self, event: sismic.model.Event):
        if event.name == "animation":
            self._animation = str(event.data.get("animation"))
        else:
            pass

    def pre(self, in_data: InData, work_data: WorkData, i: int, events: typing.Collection[sismic.model.Event] = ()):

        for event in (*events, "tick"):
            self._statemachine.queue(event).execute_once()

        if self._destination is not None:
            in_data.agents[i].destination.x, in_data.agents[i].destination.y, in_data.agents[i].destination.z = self._destination

    def post(self, in_data: InData, work_data: WorkData, i: int):
        work_data.social_state[i] = self._animation

        my = in_data.agents[i]

        if np.linalg.norm(utils.msg_to_vec(my.destination) - utils.msg_to_vec(my.pose.position)) < 0.1:
            a = utils.msg_to_vec(my.acceleration)
            a /= np.linalg.norm(a) or 1
            a *= 0.01
            work_data.force[i] = a
            work_data.vmax[i] = 0.01

    def semantic(self) -> typing.Dict:
        return {
            **{utils.SemanticAttribute(k):v for k,v in self._state.items()},
            utils.SemanticAttribute.SOCIAL_STATE: (self._social_state << 8) | self.MAGIC_NUMBER
        }

from .main import PedsimStates #noqa