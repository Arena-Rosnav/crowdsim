from .human import Human

from crowdsim_agents.utils import NormalDist as ND, ConstantDist as CD

class Elder(Human):
    _random_config = Human._random_config.apply(dict(

        min_time_in_state = +2, # little slower

        # stress_lo = 0,
        # stress_hi = 0,

        energy_lo = +2,     # tires more easily
        energy_hi = CD(1),  # never run

        social_lo = -2,     # ever got stuck chatting for 4 hours?
        social_hi = +2,

        # drift
        d_stress = -3,  # poor heart
        d_energy = -3,  # should have done more cardio...
        d_social = +3,  # at least this works

        # vision
        vision_range = -3, # could be you in 5 years 
        vision_angle = -1,

        # fears and norms
        personal_space_radius = +2,         # different times
        fear_robot = +2,                    # but not xenophobic
        fear_robot_speed_tolerance = -2,    # damn speeders
    ))