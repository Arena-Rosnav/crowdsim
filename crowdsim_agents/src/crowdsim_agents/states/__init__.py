import colorsys
import sismic.model
import sismic.io
import sismic.interpreter

import dynamic_reconfigure.client

import typing

import numpy as np
import crowdsim_agents.utils as utils

import rospy
import pedsim_msgs.msg as pedsim_msgs
import visualization_msgs.msg as visualization_msgs
import geometry_msgs.msg as geometry_msgs
import std_msgs.msg as std_msgs

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

    _pub_marker: rospy.Publisher

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

        self._pub_marker = rospy.Publisher("crowdsim_states", visualization_msgs.Marker, queue_size=None)

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

    def pre(self, in_data: utils.InData, work_data: utils.WorkData, i: int, events: typing.Collection[sismic.model.Event] = ()):

        for event in (*events, "tick"):
            self._statemachine.queue(event).execute_once()

        if self._destination is not None:
            in_data.agents[i].destination.x, in_data.agents[i].destination.y, in_data.agents[i].destination.z = self._destination

    def post(self, in_data: utils.InData, work_data: utils.WorkData, i: int):
        work_data.social_state[i] = self._animation

        my = in_data.agents[i]

        if np.linalg.norm(utils.msg_to_vec(my.destination) - utils.msg_to_vec(my.pose.position)) < 0.1:
            a = utils.msg_to_vec(my.acceleration)
            a /= np.linalg.norm(a) or 1
            a *= 0.01
            work_data.force[i] = a
            work_data.vmax[i] = 0.01

        self.visualize(in_data, i)

    def visualize(self, in_data: utils.InData, i: int):

        BAR_HEIGHT = 0.08
        BAR_WIDTH = .8
        Y_OFFSET = -0.5

        my = in_data.agents[i]

        offset = 0
        
        bars_marker = visualization_msgs.Marker()
        bars_marker.header = in_data.header
        bars_marker.ns = str(i)
        bars_marker.action = visualization_msgs.Marker.MODIFY
        bars_marker.pose.position = my.pose.position
        bars_marker.pose.orientation.w = 1
        bars_marker.lifetime = rospy.Duration(1)
        
        bars_marker.type = visualization_msgs.Marker.LINE_LIST
        bars_marker.scale.x = BAR_HEIGHT

        total = len(self._state)
        for state, value in self._state.items():
            offset += 1
            bars_marker.id = offset
            bars_marker.color = std_msgs.ColorRGBA(*colorsys.hsv_to_rgb(offset/total, .9, .9),1)
            
            bars_marker.points = []
            bars_marker.points.append(geometry_msgs.Point(BAR_WIDTH/2, Y_OFFSET-offset*BAR_HEIGHT, 0))
            bars_marker.points.append(geometry_msgs.Point(-BAR_WIDTH*(value-.5), Y_OFFSET-offset*BAR_HEIGHT, 0))
            
            self._pub_marker.publish(bars_marker)

        text_marker = visualization_msgs.Marker()
        text_marker.header = in_data.header
        text_marker.ns = str(i)
        text_marker.action = visualization_msgs.Marker.MODIFY
        text_marker.pose.position = my.pose.position
        text_marker.pose.orientation.w = 1
        text_marker.lifetime = rospy.Duration(1)
        
        text_marker.id = 0
        text_marker.type = visualization_msgs.Marker.TEXT_VIEW_FACING
        text_marker.color = std_msgs.ColorRGBA(0,0,0,1)
        text_marker.scale.x = text_marker.scale.y = text_marker.scale.z = 0.5
        text_marker.text = self._animation
        self._pub_marker.publish(text_marker)

    def semantic(self) -> typing.Dict:
        return {
            **{utils.SemanticAttribute(k):v for k,v in self._state.items()},
            utils.SemanticAttribute.SOCIAL_STATE: (self._social_state << 8) | self.MAGIC_NUMBER
        }

from .main import PedsimStates #noqa