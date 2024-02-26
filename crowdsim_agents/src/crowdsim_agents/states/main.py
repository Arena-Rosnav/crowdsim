

import json
import typing
from crowdsim_agents.states import Agent
from crowdsim_agents.states.human.adult import Adult
from crowdsim_agents.states.human.elder import Elder
from crowdsim_agents.utils import InData, WorkData
import pedsim_msgs.msg as pedsim_msgs

#TODO rework this using a registry and class method chaining
def _get_agent_class(agent_type: str) -> typing.Type[Agent]:
    if agent_type == "human/adult": return Adult
    if agent_type == "human/elder": return Elder
    return Adult

def _agent_to_index(agent_type: str) -> int:
    if agent_type == "human/adult": return 0
    if agent_type == "human/elder": return 1
    return -1


class PedsimStates:

    _agents: typing.Dict[str, Agent]


    def __init__(self):
        self._agents = dict()

    def reset(self):
        self._agents.clear()        

    def pre(self, in_data: InData, work_data: WorkData):
        for i, ped in enumerate(in_data.agents):
            if ped.id not in self._agents:
                self._agents[ped.id] = _get_agent_class(ped.type)(ped.id).setup(json.loads(ped.configuration or "{}"))
            machine = self._agents[ped.id]
            
            machine.pre(in_data, work_data, i)

    def post(self, in_data: InData, work_data: WorkData):
        for i, ped in enumerate(in_data.agents):
            machine = self._agents[ped.id]
            machine.post(in_data, work_data, i)
            
    def semantic(self) -> typing.Collection[typing.Dict[str, float]]:
        return [machine.semantic() for machine in self._agents.values()]