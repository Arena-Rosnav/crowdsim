from crowdsim_agents.forces import Forcemodel, Forcemodels
from crowdsim_agents.utils import InData

import pedsim_msgs.msg

import numpy as np
from pathlib import Path
from typing import List
import random
import rospy


@Forcemodels.register(Forcemodel.Name.PYSOCIAL)
class Plugin_PySocialForce(Forcemodel):
    FACTOR = 0.3
    GROUP_DEST_DIST = 0.5

    def __init__(self):
        global psf

        import sys

        sys.path.append(str(Path(__file__).resolve().parent))
        import pysocialforce as psf

        self.groups = dict()
        self.group_count = 0

    def assign_groups(
        self,
        agents: dict,
        _groups: List[pedsim_msgs.msg.AgentGroup],
    ) -> list:
        if not agents:
            rospy.logwarn("No agents provided")
            return []

        # First pass: Clean up stale entries
        stale_agents = [aid for aid in self.groups if aid not in agents]
        for aid in stale_agents:
            del self.groups[aid]

        # Second pass: Assign new agents to groups
        for agent_id, idx in agents.items():
            if agent_id not in self.groups:
                assigned_group = random.randint(0, self.group_count)
                if assigned_group == self.group_count:
                    self.group_count += 1
                self.groups[agent_id] = assigned_group

        # Form group list with validation using a copy
        new_groups = [list() for _ in range(self.group_count)]
        # rospy.loginfo(f"Groups: {self.groups}, Group Count: {self.group_count}")

        # Use .copy() to prevent RuntimeError during iteration
        for agent_id, group_idx in self.groups.copy().items():
            try:
                if agent_id in agents and group_idx < len(new_groups):
                    new_groups[group_idx].append(agents[agent_id])
                else:
                    del self.groups[agent_id]
                    rospy.logwarn(
                        f"Invalid agent {agent_id} or group index {group_idx}"
                    )
            except Exception as e:
                rospy.logerr(f"Error processing agent {agent_id}: {str(e)}")
                continue

        # Remove empty groups and update count
        new_groups = [group for group in new_groups if group]
        self.group_count = len(new_groups)

        if not new_groups:
            rospy.logwarn("No valid groups formed")
            return []

        return new_groups

    def overwrite_group_dest(
        self,
        state: np.ndarray,
        groups: List,
    ) -> np.ndarray:

        for group in groups:
            leader = group[0]

            for i in range(1, len(group)):
                member = group[i]

                row = (i // 8) + 1
                x_off = (
                    (1 if (i + 1) % 8 < 3 else (-1 if 3 < (i + 1) % 8 < 7 else 0))
                    * row
                    * self.GROUP_DEST_DIST
                )
                y_off = (
                    (1 if i % 8 > 4 else (-1 if 0 < i % 8 < 4 else 0))
                    * row
                    * self.GROUP_DEST_DIST
                )
                state[member, 4] = state[leader, 4] + x_off
                state[member, 5] = state[leader, 5] + y_off

        return state

    @staticmethod
    def get_state_data(
        agents: List[pedsim_msgs.msg.AgentState],
        groups: List[pedsim_msgs.msg.AgentGroup],
    ) -> np.ndarray:

        idx_assignment = dict()
        state_data = list()
        for idx in range(len(agents)):
            agent = agents[idx]

            p_x = agent.pose.position.x
            p_y = agent.pose.position.y
            v_x = agent.twist.linear.x
            v_y = agent.twist.linear.y
            d_x = agent.destination.x - p_x
            d_y = agent.destination.y - p_y

            state_data.append([p_x, p_y, v_x, v_y, d_x, d_y])
            idx_assignment[agent.id] = idx
        state = np.array(state_data)

        return state, idx_assignment

    @staticmethod
    def extract_obstacles(obstacles: List[pedsim_msgs.msg.Wall]) -> List[List[int]]:
        obs_list = list()

        for obs in obstacles:
            x_min = min(obs.start.x, obs.end.x)
            x_max = max(obs.start.x, obs.end.x)
            y_min = min(obs.start.y, obs.end.y)
            y_max = max(obs.start.y, obs.end.y)

            obs_list.append([x_min, x_max, y_min, y_max])

        return obs_list

    def compute(self, in_data, work_data):
        if len(in_data.agents) < 1:
            return

        state, agent_idx = self.get_state_data(in_data.agents, in_data.groups)
        groups = self.assign_groups(agent_idx, in_data.groups)
        state = self.overwrite_group_dest(state, groups)
        obs = self.extract_obstacles(in_data.walls)

        rospy.logdebug("Assigned Groups: " + groups.__str__())

        simulator = psf.Simulator(
            state=state,
            groups=groups,
            obstacles=obs,
            config_file=Path(__file__)
            .resolve()
            .parent.joinpath("pysocialforce/config/default.toml"),
        )

        forces = self.FACTOR * simulator.compute_forces()

        work_data.force[:, [0, 1]] = forces
        work_data.force[np.isnan(work_data.force)] = 0
