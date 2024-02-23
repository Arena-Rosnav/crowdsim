from crowdsim_agents.forces import Forcemodel, Forcemodels
import crowdsim_agents.utils

import pedsim_msgs.msg

@Forcemodels.register(Forcemodel.Name.DEEPSOCIALFORCE)
class Plugin_DSF(Forcemodel):

    steps = 1
    buffer_length = 7

    def __init__(self):

        from socialforce import Simulator
        import socialforce.potentials

        self.simulator = Simulator(ped_ped=socialforce.potentials.PedPedPotential(sigma=1))
        # self.old_forces = dict()
        # self.force_buffers = dict()
        self.start = True

    def compute(self, in_data, work_data):
        agent_ids = []
        agent_states = []
        for agent in in_data.agents:
            agent_ids.append(agent.id)
            agent_states.append((agent.pose.position.x, 
                           agent.pose.position.y, 
                           agent.twist.linear.x, 
                           agent.twist.linear.y, 
                           agent.acceleration.x, 
                           agent.acceleration.y, 
                           agent.destination.x, 
                           agent.destination.y, 
                           agent.direction,
                           7)) #desired speed???
        
        if agent_states == []:
            self.start = True
            return []
        
        state = self.simulator.normalize_state(agent_states)
        
        if self.start:
            # self.force_buffers = {agent_id:ForceBuffer(self.buffer_length) for agent_id in agent_ids}
            # self.old_forces = {agent_id:Force() for agent_id in agent_ids}
            states = self.simulator.run(state,self.buffer_length)
            # for state in states:
            #     for i,agent_id in enumerate(agent_ids):
            #         self.force_buffers[agent_id].apply(Force(state[i][4],state[i][5]))
            #         self.force_buffers[agent_id].pop()
            self.start = False
            return []
        
        states = self.simulator.run(state,self.steps)
        next_state = states[-1]

        feedbacks = []

        work_data.force[[0,1]] = next_state[:,[4,5]]

        # for i, agent_id in enumerate(agent_ids):
        #     state = next_state[i]
        #     feedback = pedsim_msgs.msg.AgentFeedback()
        #     feedback.id = agent_id

        #     force_new = Force(state[4],state[5])
        #     #forces_diff = force_new - self.old_forces[agent_id]
        #     force_pub = self.force_buffers[agent_id].pop()
        #     self.force_buffers[agent_id].apply(force_new)

            
    
# class Force:
    
#     def __init__(self, x=0,y=0):
#         self.x = x
#         self.y = y
    
#     def __add__(self,force):
#         return Force(self.x+force.x, self.y+force.y)
    
#     def __sub__(self,force):
#         return self + force*(-1)
    
#     def __mul__(self, scale):
#         return Force(self.x*scale, self.y*scale)
    
#     def __iadd__(self,force):
#         self.x += force.x
#         self.y += force.y
#         return self
    
#     def __truediv__(self, scale):
#         return self*(1/scale)
    
#     def __eq__(self, force):
#         return self.x == force.x and self.y == force.y
    
# class ForceBuffer:

#     def __init__(self, size: int):
#         self.__buffer = [Force() for _ in range(size)]

#     def pop(self) -> Force:
#         self.__buffer.append(Force())
#         return self.__buffer.pop(0)

#     def apply(self, force: Force):
#         for buffer_force in self.__buffer:
#             buffer_force += force / len(self.__buffer)
#         # extension: scale force non-constantly based on position with for i, buffer_force in enumerate(self.__buffer)
    
#     def add(self, force: Force):
#         for buffer_force in self.__buffer:
#             buffer_force = (buffer_force+force)/2
  


