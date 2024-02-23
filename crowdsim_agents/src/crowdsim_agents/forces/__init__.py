from enum import Enum
from typing import Dict, Type

from crowdsim_agents.utils import InData, WorkData
import rospy

class Forcemodel:

    class Name(Enum):
        PASSTHROUGH = "passthrough"
        SPINNY = "spinny"
        PYSOCIAL = "pysocial"
        DEEPSOCIALFORCE = "deepsocialforce"
        EVACUATION = "evacuation"

    def compute(self, in_data: InData, work_data: WorkData):
        raise NotImplementedError()

class Forcemodels:

    __registry: Dict[Forcemodel.Name, Type[Forcemodel]] = dict()

    @classmethod
    def register(cls, name: Forcemodel.Name):
        def inner(force_model: Type[Forcemodel]):
            if cls.__registry.get(name) is not None:
                raise NameError(f"force model {name} is already registered")

            cls.__registry[name] = force_model
            return force_model
        return inner

    forcemodel_name: Forcemodel.Name
    forcemodel_class: Type[Forcemodel]
    forcemodel: Forcemodel

    publisher: rospy.Publisher
    running: bool

    def __init__(self, name: str):

        try:
            forcemodel_name = Forcemodel.Name(name)
        except ValueError as e:
            raise ValueError(f"Force model {name} does not exist.\nAvailable force models: {[name.value for name in Forcemodel.Name]}") from e

        self.forcemodel_name = forcemodel_name

        forcemodel_class = self.__registry.get(forcemodel_name)

        if forcemodel_class is None:
            raise RuntimeError(f"Force model {forcemodel_name.value} has no registered implementation.\nImplemented force models: {[name.value for name in self.__registry.keys()]}")

        self.forcemodel_class = forcemodel_class

        self.running = False


    def reset(self):
        if self.running:
            del self.forcemodel 
            self.running = False

        try:
            self.forcemodel: Forcemodel = self.forcemodel_class()
            rospy.loginfo(f"starting crowdsim_agents with force model {self.forcemodel_class.__name__}")
        except Exception as e:
            rospy.signal_shutdown(f"Could not initialize force model {self.forcemodel_name.value}. Aborting.")
            raise RuntimeError(f"Could not initialize force model {self.forcemodel_name.value}. Aborting.") from e
        
        self.running = True

    def run(self, in_data: InData, work_data: WorkData):

        if len(in_data.agents) == 0: return
        return self.forcemodel.compute(in_data=in_data, work_data=work_data)