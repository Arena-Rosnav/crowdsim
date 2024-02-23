from crowdsim_agents.forces import Forcemodel, Forcemodels

import pedsim_msgs.msg

@Forcemodels.register(Forcemodel.Name.PASSTHROUGH)
class Plugin_Passthrough(Forcemodel):

    def __init__(self):
        ...

    def compute(self, in_data, work_data):
        ...