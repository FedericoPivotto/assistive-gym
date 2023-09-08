import numpy as np
import pybullet as p

from .env import AssistiveEnv

# The robot handovers a wrench through its end effector to the human's right hand
class ObjectHandoverEnv(AssistiveEnv):
    def __init__(self, robot, human):
        super(ObjectHandoverEnv, self).__init__(robot=robot, human=human, task='object_handover', obs_robot_len=(23 + len(robot.controllable_joint_indices) - (len(robot.wheel_joint_indices) if robot.mobile else 0)), obs_human_len=(24 + len(human.controllable_joint_indices)))

    def set_human_tool(self):
        # Disable the robot end effector tool constraint to pass the tool to the human
        self.disable_constraint(self.tool.constraint)
        # Create the constraint that keeps the tool in the right hand of the human
        self.create_human_right_hand_tool_constraint()
        # Enable the human right hand tool constraint to make the human grasp the tool
        self.enable_constraint(self.human_tool_constraint)

    def create_human_right_hand_tool_constraint(self):
        # Define the human right hand tool constraint that keeps the tool in the right hand of the human
        self.human_tool_constraint = p.createConstraint(self.human.body, self.human.right_wrist, self.tool.body, -1, p.JOINT_FIXED, [0, 0, 0], parentFramePosition=self.hand_pos_offset, childFramePosition=[0, 0, 0], parentFrameOrientation=[0, 0, 0], childFrameOrientation=[0, 0, 0, 1])
    
    def enable_constraint(self, constraintId, maxForce=500):
        # Enable the constraining force
        p.changeConstraint(userConstraintUniqueId=constraintId, maxForce=maxForce)

    def disable_constraint(self, constraintId):
        # Disable the constraining force
        self.enable_constraint(constraintId, 0)

    def step(self, action):
        if self.human.controllable:
            action = np.concatenate([action['robot'], action['human']])
        self.take_step(action)  

        # Set human and robot observations
        obs = self._get_obs()

        # Get human preferences
        end_effector_velocity = np.linalg.norm(self.robot.get_velocity(self.robot.left_end_effector))
        preferences_score = self.human_preferences(end_effector_velocity=end_effector_velocity, total_force_on_human=self.total_force_on_human, tool_force_at_target=self.tool_force_at_target)

        # Action reward
        reward_action = -np.linalg.norm(action) # Penalize actions

        # Object handover task
        tool_pos = self.tool.get_base_pos_orient()[0]
        distance_to_hand = np.linalg.norm(self.target_pos - tool_pos)
        # Rewards for object handover
        reward_distance = -distance_to_hand if self.task_success == 0 else 0.0 # Penalize distances away from target
        reward_handover = 0.0 # Initial reward for completing handover task
        # Check whether the end effector is sufficiently close to the human's right hand
        if self.task_success == 0 and distance_to_hand < 0.1:
            # Remove tool from robot end effector to handover it to the human's right hand
            self.set_human_tool()
            # High reward for object handover
            reward_handover = 10.0
            # Object handover task was successful
            self.task_success += 1

        # Robot end effector reposition task
        ee_pos = self.robot.get_pos_orient(self.robot.right_tool_joint)[0]
        distance_to_start = np.linalg.norm(self.start_ee_pos - ee_pos)
        # Rewards for robot end effector reposition
        reward_distance_start = -distance_to_start if self.task_success > 0 else 0.0 # Penalize distances away from target
        reward_reposition = 0.0 # Initial reward for completing reposition task
        # Check whether the end effector is sufficiently close to the starting position
        if self.task_success == 1 and distance_to_start < 0.1:
            # High reward for end effector reposition
            reward_reposition = 10.0
            # Robot end effector reposition task was successful
            self.task_success += 1

        # Total reward assigned to the current timestep
        reward = self.config('distance_weight')*(reward_distance + reward_distance_start) + self.config('action_weight')*reward_action + self.config('task_reward_weight')*(reward_handover + reward_reposition) + preferences_score

        if self.gui and self.tool_force_at_target > 0:
            print('Task success:', self.task_success, 'Tool at target:', self.reward_handover, 'End effector at start:', reward_reposition)

        info = {'total_force_on_human': self.total_force_on_human, 'task_success': int(self.task_success >= self.config('task_success_threshold')), 'action_robot_len': self.action_robot_len, 'action_human_len': self.action_human_len, 'obs_robot_len': self.obs_robot_len, 'obs_human_len': self.obs_human_len}
        done = self.iteration >= 200

        if not self.human.controllable:
            return obs, reward, done, info
        else:
            # Co-optimization with both human and robot controllable
            return obs, {'robot': reward, 'human': reward}, {'robot': done, 'human': done, '__all__': done}, {'robot': info, 'human': info}
    

    def get_total_force(self):
        total_force_on_human = np.sum(self.robot.get_contact_points(self.human)[-1])
        tool_force = np.sum(self.tool.get_contact_points()[-1])
        tool_force_at_target = 0
        target_contact_pos = None
        for linkA, linkB, posA, posB, force in zip(*self.tool.get_contact_points(self.human)):
            total_force_on_human += force
            # Enforce that contact is close to the target location
            if linkA in [0, 1] and np.linalg.norm(posB - self.target_pos) < 0.025:
                tool_force_at_target += force
                target_contact_pos = posB
        return total_force_on_human, tool_force, tool_force_at_target, None if target_contact_pos is None else np.array(target_contact_pos)

    def _get_obs(self, agent=None):
        tool_pos, tool_orient = self.tool.get_base_pos_orient()
        tool_pos_real, tool_orient_real = self.robot.convert_to_realworld(tool_pos, tool_orient)
        robot_joint_angles = self.robot.get_joint_angles(self.robot.controllable_joint_indices)
        # Fix joint angles to be in [-pi, pi]
        robot_joint_angles = (np.array(robot_joint_angles) + np.pi) % (2*np.pi) - np.pi
        if self.robot.mobile:
            # Don't include joint angles for the wheels
            robot_joint_angles = robot_joint_angles[len(self.robot.wheel_joint_indices):]
        shoulder_pos = self.human.get_pos_orient(self.human.right_shoulder)[0]
        elbow_pos = self.human.get_pos_orient(self.human.right_elbow)[0]
        wrist_pos = self.human.get_pos_orient(self.human.right_wrist)[0]
        shoulder_pos_real, _ = self.robot.convert_to_realworld(shoulder_pos)
        elbow_pos_real, _ = self.robot.convert_to_realworld(elbow_pos)
        wrist_pos_real, _ = self.robot.convert_to_realworld(wrist_pos)
        target_pos_real, _ = self.robot.convert_to_realworld(self.target_pos)
        self.total_force_on_human, self.tool_force, self.tool_force_at_target, self.target_contact_pos = self.get_total_force()
        robot_obs = np.concatenate([tool_pos_real, tool_orient_real, tool_pos_real - target_pos_real, target_pos_real, robot_joint_angles, shoulder_pos_real, elbow_pos_real, wrist_pos_real, [self.tool_force]]).ravel()
        if agent == 'robot':
            return robot_obs
        if self.human.controllable:
            human_joint_angles = self.human.get_joint_angles(self.human.controllable_joint_indices)
            tool_pos_human, tool_orient_human = self.human.convert_to_realworld(tool_pos, tool_orient)
            shoulder_pos_human, _ = self.human.convert_to_realworld(shoulder_pos)
            elbow_pos_human, _ = self.human.convert_to_realworld(elbow_pos)
            wrist_pos_human, _ = self.human.convert_to_realworld(wrist_pos)
            target_pos_human, _ = self.human.convert_to_realworld(self.target_pos)
            human_obs = np.concatenate([tool_pos_human, tool_orient_human, tool_pos_human - target_pos_human, target_pos_human, human_joint_angles, shoulder_pos_human, elbow_pos_human, wrist_pos_human, [self.total_force_on_human, self.tool_force_at_target]]).ravel()
            if agent == 'human':
                return human_obs
            # Co-optimization with both human and robot controllable
            return {'robot': robot_obs, 'human': human_obs}
        return robot_obs

    def generate_target(self):
        # Set the target position on the human's right hand based on an offset from the right wrist
        self.hand_pos_offset = [0, 0, -0.05]
        wrist_pos, wrist_orient = self.human.get_pos_orient(self.human.right_wrist)
        target_pos, target_orient = p.multiplyTransforms(wrist_pos, wrist_orient, self.hand_pos_offset, [0, 0, 0, 1], physicsClientId=self.id)
        
        # Create target object
        self.target = self.create_sphere(radius=0.01, mass=0.0, pos=target_pos, visual=True, collision=False, rgba=[0, 1, 1, 1])
        
        self.update_targets()

    def update_targets(self):
        wrist_pos, wrist_orient = self.human.get_pos_orient(self.human.right_wrist)
        target_pos, target_orient = p.multiplyTransforms(wrist_pos, wrist_orient, self.hand_pos_offset, [0, 0, 0, 1], physicsClientId=self.id)
        
        # Save the initial target position and orientation
        self.target_pos = np.array(target_pos)
        self.target_orient = np.array(target_orient)
        
        # Set target position and orientation
        self.target.set_base_pos_orient(self.target_pos, [0, 0, 0, 1])

    def reset(self):
        super(ObjectHandoverEnv, self).reset()
        # Build an environment with a toolbox
        self.build_assistive_env('toolbox')

        # Set camera position
        p.resetDebugVisualizerCamera(cameraDistance=0.6, cameraYaw=40, cameraPitch=-30, cameraTargetPosition=[-0.18, 0, 1.15], physicsClientId=self.id)

        # Set human initial position and orientation
        human_pos, human_orient = [-0.1, 0.5, 0.7], [0, 0, -np.pi/4.0]
        self.human.set_base_pos_orient(human_pos, human_orient)

        # Set joint angles in degrees for human joints
        # Human on his knees with elbows bent and forearms raised
        joints_positions = [(self.human.j_right_elbow, -180), (self.human.j_left_elbow, -180), (self.human.j_right_hip_x, 0), (self.human.j_right_knee, 170), (self.human.j_left_hip_x, 0), (self.human.j_left_knee, 170)]
        # Head orientation
        self.human.setup_joints(joints_positions, use_static_joints=True, reactive_force=None if self.human.controllable else 1, reactive_gain=0.01)

        shoulder_pos = self.human.get_pos_orient(self.human.right_shoulder)[0]
        elbow_pos = self.human.get_pos_orient(self.human.right_elbow)[0]
        wrist_pos = self.human.get_pos_orient(self.human.right_wrist)[0]

        # Initialize the tool in the robot's gripper and set tool mesh scale
        self.tool.init(self.robot, self.task, self.directory, self.id, self.np_random, right=True, mesh_scale=[0.035]*3)

        # Set initial end effector position away from human's right hand
        target_ee_pos = np.array([-0.4, -0.1, 0.8]) + self.np_random.uniform(-0.05, 0.05, size=3)
        target_ee_orient = self.get_quaternion(self.robot.toc_ee_orient_rpy[self.task])
        self.init_robot_pose(target_ee_pos, target_ee_orient, [(target_ee_pos, target_ee_orient)], [(shoulder_pos, None), (elbow_pos, None), (wrist_pos, None)], arm='left', tools=[self.tool], collision_objects=[self.human, self.furniture])

        # Open gripper to hold the tool according to the robot
        self.robot.set_gripper_open_position(self.robot.left_gripper_indices, self.robot.gripper_pos[self.task], set_instantly=True)

        # Starting position of the robot's end effector
        self.start_ee_pos = self.robot.get_pos_orient(self.robot.right_tool_joint)[0]

        # Generate target on human's right hand
        self.generate_target()

        if not self.robot.mobile:
            self.robot.set_gravity(0, 0, 0)
        self.human.set_gravity(0, 0, 0)
        self.tool.set_gravity(0, 0, 0)

        # Enable rendering
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1, physicsClientId=self.id)

        self.init_env_variables()
        return self._get_obs()
