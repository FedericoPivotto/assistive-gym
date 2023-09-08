import os
import numpy as np
import pybullet as p
from .robot import Robot

class TIAGo(Robot):
    def __init__(self, controllable_joints='right'):
        right_arm_joint_indices = [35, 36, 37, 38, 39, 40, 41] # Controllable arm joints
        left_arm_joint_indices = right_arm_joint_indices # Controllable arm joints
        wheel_joint_indices = [12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23] # Controllable wheel, caster and suspension joints
        right_end_effector = 45 # Used to get the pose of the end effector
        left_end_effector = right_end_effector # Used to get the pose of the end effector
        right_gripper_indices = [51, 52] # Gripper actuated joints
        left_gripper_indices = right_gripper_indices # Gripper actuated joints
        right_tool_joint = 45 # Joint that tools are attached to
        left_tool_joint = right_tool_joint # Joint that tools are attached to
        
        # NOTE: torso lift joint is fixed in tiago.urdf

        right_gripper_collision_indices = list(range(43,53)) # Used to disable collision between gripper and tools
        left_gripper_collision_indices = right_gripper_collision_indices # Used to disable collision between gripper and tools

        # Gripper open position for holding tools
        gripper_pos = {'scratch_itch': [0.025, 0.025],
                       'feeding': [0.01, 0],
                       'drinking': [0.037, 0.037],
                       'bed_bathing': [0.025, 0.025],
                       'dressing': [0]*2,
                       'arm_manipulation': [0.025, 0.025],
                       'object_handover': [0.007, 0.007]}
        
        # Position offset between tool and robot tool joint
        tool_pos_offset = {'scratch_itch': [0, 0, -0.2],
                           'feeding': [0.01, -0.02, -0.28],
                           'drinking': [0, -0.03, -0.16],
                           'bed_bathing': [0, 0, -0.2],
                           'dressing': [],
                           'arm_manipulation': [0, -0.08, -0.31],
                           'object_handover': [-0.015, 0, -0.2]}
        
        # RPY orientation offset between tool and robot tool joint
        tool_orient_offset = {'scratch_itch': [0, np.pi/2.0, 0],
                              'feeding': [-0.2, 0, 0],
                              'drinking': [0, np.pi/2.0, 0],
                              'bed_bathing': [-np.pi/2, 0, 0],
                              'dressing': [],
                              'arm_manipulation': [0, np.pi/2, 0],
                              'object_handover': [np.pi/2.0, np.pi, np.pi/2.0]}
        
        # Robot base offset before TOC base pose optimization
        toc_base_pos_offset = {'scratch_itch': [-0.2, -0.13, 0.05],
                               'feeding': [0.1, 0.2, 0],
                               'drinking': [0.2, -0.08, 0.05],
                               'bed_bathing': [-0.1, 0, 0],
                               'dressing': [1.7, 0.7, 0],
                               'arm_manipulation': [-0.5, 0.7, 0],
                               'object_handover': [-0.1, 0.5, 0.05]} 

        # Initial end effector orientation
        toc_ee_orient_rpy = {'scratch_itch': [np.pi/2.0, 0, 0],
                             'feeding': [np.pi/2.0, 0, 0],
                             'drinking': [np.pi/2.0, 0, 0],
                             'bed_bathing': [np.pi/2.0, 0, 0],
                             'dressing': [[0, 0, np.pi], [0, 0, np.pi*3/2.0]],
                             'arm_manipulation': [np.pi/2, 0, 0],
                             'object_handover': [np.pi/2.0, 0, 0]}
        
        # NOTE: in BedBathing environments, sometimes the arm glitches
        # NOTE: in ArmManipulation environments, after the first run the human model swells
        
        wheelchair_mounted = False

        super(TIAGo, self).__init__(controllable_joints, right_arm_joint_indices, left_arm_joint_indices, wheel_joint_indices, right_end_effector, left_end_effector, right_gripper_indices, left_gripper_indices, gripper_pos, right_tool_joint, left_tool_joint, tool_pos_offset, tool_orient_offset, right_gripper_collision_indices, left_gripper_collision_indices, toc_base_pos_offset, toc_ee_orient_rpy, wheelchair_mounted, half_range=True)

    def init(self, directory, id, np_random, fixed_base=True):
        self.body = p.loadURDF(os.path.join(directory, 'tiago', 'tiago.urdf'), useFixedBase=fixed_base, basePosition=[-1, -1, 0], flags=p.URDF_USE_SELF_COLLISION, physicsClientId=id)
        super(TIAGo, self).init(self.body, id, np_random)