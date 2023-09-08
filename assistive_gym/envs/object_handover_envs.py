from .object_handover import ObjectHandoverEnv
from .agents import tiago, human
from .agents.tiago import TIAGo

from .agents.human import Human
from ray.rllib.env.multi_agent_env import MultiAgentEnv
from ray.tune.registry import register_env

robot_arm = 'right'
human_controllable_joint_indices = human.right_arm_joints # Human active joints

class ObjectHandoverTIAGoEnv(ObjectHandoverEnv):
    def __init__(self):
        super(ObjectHandoverTIAGoEnv, self).__init__(robot=TIAGo(robot_arm), human=Human(human_controllable_joint_indices, controllable=False))

class ObjectHandoverTIAGoHumanEnv(ObjectHandoverEnv, MultiAgentEnv):
    def __init__(self):
        super(ObjectHandoverTIAGoHumanEnv, self).__init__(robot=TIAGo(robot_arm), human=Human(human_controllable_joint_indices, controllable=True))
register_env('assistive_gym:ObjectHandoverTIAGoHuman-v1', lambda config: ObjectHandoverTIAGoHumanEnv())