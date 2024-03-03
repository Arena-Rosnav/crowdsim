import enum
import functools
import itertools
import os
import typing

import sismic.model
import numpy as np

import crowdsim_agents.utils as utils

from .. import Agent, StatechartProvider

from crowdsim_agents.utils import NormalDist as ND, ConstantDist as CD
SND = lambda mean, stdev: ND(mean, stdev, 0, 1)

class _States:
    STRESS = utils.SemanticAttribute.STATE_STRESS.value
    ENERGY = utils.SemanticAttribute.STATE_ENERGY.value
    SOCIAL = utils.SemanticAttribute.STATE_SOCIAL.value

@StatechartProvider.load(os.path.join(os.path.dirname(__file__), "human.yaml"))
class Human(Agent):

    MAGIC_NUMBER = 0x01

    _tracked_agents: typing.Set[str]
    _tracked_obstacles: typing.Set[str]
    _seek: typing.Optional[str]

    _random_config = utils.RandomConfig(
        **Agent._random_config,

        min_time_in_state = ND(2.0, 0.5, 1.0, None), # seconds

        stress_lo = SND(0.20, 0.05),
        stress_hi = SND(0.80, 0.05),

        energy_lo = SND(0.20, 0.05),
        energy_hi = SND(0.80, 0.05),

        social_lo = SND(0.20, 0.05),
        social_hi = SND(0.80, 0.05),

        # drift
        d_stress = ND(-0.01, 0.001, None, -0.001),
        d_energy = ND(+0.01, 0.001, +0.001, None),
        d_social = ND(+0.01, 0.001, +0.001, None),

        # idle state [rest]
        idle_d_stress = ND(-0.05, 0.001),
        idle_d_energy = ND(+0.15, 0.001),
        idle_d_social = ND(+0.05, 0.001),

        # walking state [rest]
        walking_d_stress = ND(+0.00, 0.001),
        walking_d_energy = ND(+0.00, 0.001),
        walking_d_social = ND(+0.00, 0.001),

        # running state
        running_d_stress = ND(+0.00, 0.01),
        running_d_energy = ND(-0.05, 0.01),
        running_d_social = ND(+0.00, 0.01),

        # talking state
        talking_d_stress = ND(+0.00, 0.01),
        talking_d_energy = ND(+0.00, 0.01),
        talking_d_social = ND(-0.05, 0.01),

        # phone state
        phone_d_stress = ND(+0.00, 0.01),
        phone_d_energy = ND(+0.00, 0.01),
        phone_d_social = ND(-0.05, 0.01),

        # interacting state
        interacting_d_stress = ND(+0.00, 0.01),
        interacting_d_energy = ND(-0.05, 0.01),
        interacting_d_social = ND(+0.05, 0.01),

        # vision
        vision_range = ND(5.0, 0.5, 0, None),
        vision_angle = ND(45, 5, 0, 90),

        # fears and norms
        personal_space_radius = ND(1.0, 0.1, 0.5, None),
        fear_robot = ND(+0.02, 0.001),
        fear_robot_speed_tolerance = ND(0.7, 0.1, 0, None), #m/s = 2.5km/h
    )

    class Animation(int, enum.Enum):
        IDLE        = enum.auto()
        WALKING     = enum.auto()
        RUNNING     = enum.auto()
        INTERACTING = enum.auto()
        TALKING     = enum.auto()
        PHONE       = enum.auto()

    def __init__(self, id: str):
        super().__init__(id)

        self._tracked_agents = set()
        self._tracked_obstacles = set()
        self._seek = None

        rng = self._runtime["rng"]

        self._config = dict(
        )

        class _State(dict):

            def __init__(
                self,
                **kwargs
            ):
                for k,v in kwargs.items():
                    self.__setitem__(k,v)

            def __setitem__(self, __key, __value):
                super().__setitem__(__key, min(1, max(0, __value)))


        self._state = _State(**{
            _States.ENERGY: .50,
            _States.STRESS: .00,
            _States.SOCIAL: .50
        })

    def event_handler(self, event):
        if event.name == "animation":
            self._animation = self.Animation[event.data["animation"]].name
            self._social_state = self.Animation[event.data["animation"]].value
        
        elif event.name == "seek":
            self._seek = event.data.get("id", None)
            if self._seek is None: self._destination = None

        else:
            super().event_handler(event)

    def pre(self, in_data, work_data, i, events: typing.Collection[sismic.model.Event] = ()):
        new_events: typing.Collection[sismic.model.Event] = []

        my_position = utils.msg_to_vec(in_data.agents[i].pose.position)

        # I shall fear no robot
        for robot in in_data.robots:
            if np.linalg.norm(utils.msg_to_vec(robot.pose.position) - my_position) < self._config["personal_space_radius"]:
                self._state[_States.STRESS] += self._config["fear_robot"]
                self._state[_States.STRESS] += max(0, np.linalg.norm(utils.msg_to_vec(robot.twist.linear)) - self._config["fear_robot_speed_tolerance"])
            

        # who do I see?
        i_see = lambda z: np.logical_or(
            np.logical_and(np.abs(np.angle(z)) < np.deg2rad(self._config["vision_angle"]), np.abs(z) < self._config["vision_range"]), # vision
            np.abs(z) < 1.1 * self._config["personal_space_radius"] # perception
        )
        
        p = in_data.agents[i].pose.position
        d_p = lambda v: (v.x-p.x, v.y-p.y)

        seen_agents = set(
            in_data.agents[a].id
            for a
            in np.where(
                i_see(
                    np.array([d_p(ag.pose.position) for ag in in_data.agents]).view(dtype=np.complex128)
                )
            )[0].flat
            if a != i
        ) 

        seen_obstacles = set(
            in_data.obstacles[o].name
            for o
            in np.where(
                i_see(
                    np.array([d_p(obstacle.pose.position) for obstacle in in_data.obstacles]).view(dtype=np.complex128)
                )
            )[0].flat
        ) 

        for added in seen_agents.difference(self._tracked_agents):
            new_events.append(sismic.model.Event("agent", id=added))
        for lost in self._tracked_agents.difference(seen_agents):
            if lost == self._seek:
                new_events.append(sismic.model.Event("lost"))
        self._tracked_agents = seen_agents

        for added in seen_obstacles.difference(self._tracked_obstacles):
            new_events.append(sismic.model.Event("obstacle", id=added))
        for lost in self._tracked_obstacles.difference(seen_obstacles):
            if lost == self._seek:
                new_events.append(sismic.model.Event("lost"))
        self._tracked_obstacles = seen_obstacles

        # time to seek
        if self._seek is not None:
            hit = next(
                itertools.chain(
                    (agent for agent in in_data.agents if agent.id == self._seek),
                    (obstacle for obstacle in in_data.obstacles if obstacle.name == self._seek),
                ),
                None
            )
            if hit is not None:
                p_seek = utils.msg_to_vec(hit.pose.position)
                p_my = my_position.copy()
                
                p_my -= p_seek
                p_my /= np.linalg.norm(p_my)
                p_my *= self._config["personal_space_radius"]
                p_my += p_seek

                self._destination = p_my.flat


        super().pre(in_data, work_data, i, events=(*events, *new_events))



    def post(self, in_data, work_data, i):
        if self._animation == self.Animation.RUNNING: work_data.vmax[i] = 2.
        if self._animation == self.Animation.PHONE: work_data.vmax[i] = .5

        super().post(in_data, work_data, i)