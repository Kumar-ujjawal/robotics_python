import pybullet as p
import pybullet_data
import numpy as np
import math

class KinovaEnv:
    def __init__(self):
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        urdf_path = r"kinova-ros/kinova_description/urdf/j2s7s300_standalone.urdf"
        self.planeId = p.loadURDF("plane.urdf")
        self.robotId = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=True)
        
        self.initial_joint_positions = [0.0, 0.0, 2.9, 1.3, -2.07, 1.4, 0.0, 1.0, 1.0, 1.0]
        for i, pos in enumerate(self.initial_joint_positions):
            p.resetJointState(self.robotId, i, pos)

        for i in range(p.getNumJoints(self.robotId)):
            p.setJointMotorControl2(self.robotId, i, p.VELOCITY_CONTROL, force=0)

        self.controllable_joints = [i for i in range(p.getNumJoints(self.robotId)) if p.getJointInfo(self.robotId, i)[2] != p.JOINT_FIXED]
        
        ball_radius = 0.05
        ball_position = [0.5, 0, ball_radius]
        ball_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=ball_radius)
        ball_visual = p.createVisualShape(p.GEOM_SPHERE, radius=ball_radius, rgbaColor=[1, 0, 0, 1])
        self.ballId = p.createMultiBody(baseMass=0.1, baseCollisionShapeIndex=ball_shape, baseVisualShapeIndex=ball_visual, basePosition=ball_position)

        self.ball_position = np.array(ball_position)
        self.action_space = len(self.controllable_joints)
        self.last_distance = None
        self.steps_without_progress = 0
        self.distance_threshold = 0.001
        self.max_steps_without_progress = 10

        # New additions
        self.joint_limits = [(p.getJointInfo(self.robotId, i)[8], p.getJointInfo(self.robotId, i)[9]) for i in self.controllable_joints]
        self.obstacle_positions = [np.array([0.3, 0.3, 0.5]), np.array([-0.3, -0.3, 0.5])]  # Example obstacles
        self.curriculum_stage = 0
        self.max_curriculum_stages = 5
        self.episode_count = 0
        self.episodes_per_curriculum = 200

    def reset(self):
        for i, pos in enumerate(self.initial_joint_positions):
            p.resetJointState(self.robotId, i, pos)
        p.resetBasePositionAndOrientation(self.ballId, self.ball_position, [0, 0, 0, 1])
        self.last_distance = self._get_distance_to_ball()
        self.steps_without_progress = 0
        self.last_joint_positions = self._get_joint_positions()
        return self._get_state()

    def step(self, action):
        current_distance = self._get_distance_to_ball()
        
        if abs(current_distance - self.last_distance) < self.distance_threshold:
            self.steps_without_progress += 1
        else:
            self.steps_without_progress = 0

        if self.steps_without_progress >= self.max_steps_without_progress:
            action = self._take_random_action()
            self.steps_without_progress = 0

        for i, joint in enumerate(self.controllable_joints):
            p.setJointMotorControl2(
                bodyUniqueId=self.robotId,
                jointIndex=joint,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=action[i],
                force=1000.0
            )
        
        for _ in range(10):
            p.stepSimulation()

        state = self._get_state()
        reward = self._compute_reward()
        done = self._is_done()

        self.last_distance = current_distance
        self.last_joint_positions = self._get_joint_positions()

        return state, reward, done

    def _take_random_action(self):
        return np.random.uniform(-1, 1, self.action_space)

    def _get_state(self):
        joint_states = p.getJointStates(self.robotId, self.controllable_joints)
        joint_positions = np.array([state[0] for state in joint_states])
        joint_velocities = np.array([state[1] for state in joint_states])
        end_effector_pos, end_effector_orn = p.getLinkState(self.robotId, len(self.controllable_joints) - 1)[:2]
        return np.concatenate([joint_positions, joint_velocities, end_effector_pos, end_effector_orn])

    def _get_distance_to_ball(self):
        end_effector_pos = np.array(p.getLinkState(self.robotId, len(self.controllable_joints) - 1)[0])
        return np.linalg.norm(end_effector_pos - self.ball_position)

    def _get_joint_positions(self):
        return np.array([state[0] for state in p.getJointStates(self.robotId, self.controllable_joints)])

    def _get_joint_velocities(self):
        return np.array([state[1] for state in p.getJointStates(self.robotId, self.controllable_joints)])

    def _get_joint_torques(self):
        return np.array([state[3] for state in p.getJointStates(self.robotId, self.controllable_joints)])

    def _compute_reward(self):
        current_distance = self._get_distance_to_ball()
        current_joint_positions = self._get_joint_positions()
        current_joint_velocities = self._get_joint_velocities()
        current_joint_torques = self._get_joint_torques()
        end_effector_pos, end_effector_orn = p.getLinkState(self.robotId, len(self.controllable_joints) - 1)[:2]

        # Distance reward
        distance_reward = self.last_distance - current_distance

        # Smooth motion reward
        smoothness_reward = -np.sum(np.square(current_joint_velocities - self._get_joint_velocities()))

        # Joint limit penalty
        joint_limit_penalty = sum(min(abs(pos - limit[0]), abs(pos - limit[1])) for pos, limit in zip(current_joint_positions, self.joint_limits))

        # Obstacle avoidance
        obstacle_penalty = sum(1 / np.linalg.norm(np.array(end_effector_pos) - obs_pos) for obs_pos in self.obstacle_positions)

        # Task-specific reward (reaching the ball)
        task_reward = 10 if current_distance < 0.1 else 0

        # Energy efficiency
        energy_penalty = np.sum(np.square(current_joint_torques))

        # Orientation reward
        target_orn = p.getQuaternionFromEuler([0, 0, 0])  # Assuming we want the end-effector to be upright
        orn_diff = sum(abs(a - b) for a, b in zip(end_effector_orn, target_orn))
        orientation_reward = -orn_diff

        # Path length reward
        path_length_reward = -np.linalg.norm(current_joint_positions - self.last_joint_positions)

        # Combine rewards with weights
        w1, w2, w3, w4, w5, w6, w7, w8 = 1.0, 0.1, 0.1, 0.1, 1.0, 0.01, 0.1, 0.1
        total_reward = (
            w1 * distance_reward +
            w2 * smoothness_reward +
            w3 * joint_limit_penalty +
            w4 * obstacle_penalty +
            w5 * task_reward +
            w6 * energy_penalty +
            w7 * orientation_reward +
            w8 * path_length_reward
        )

        # Curriculum learning
        if self.curriculum_stage < self.max_curriculum_stages:
            total_reward *= (self.curriculum_stage + 1) / self.max_curriculum_stages

        # Potential-based shaping reward
        shaping_reward = self._potential_based_reward()

        return total_reward + shaping_reward

    def _potential_based_reward(self):
        current_distance = self._get_distance_to_ball()
        potential = -current_distance  # Negative distance as potential
        if hasattr(self, 'last_potential'):
            shaping_reward = potential - self.last_potential
        else:
            shaping_reward = 0
        self.last_potential = potential
        return shaping_reward

    def _is_done(self):
        return self._get_distance_to_ball() < 0.1

    def close(self):
        p.disconnect(self.physicsClient)

    def update_curriculum(self):
        self.episode_count += 1
        if self.episode_count % self.episodes_per_curriculum == 0:
            self.curriculum_stage = min(self.curriculum_stage + 1, self.max_curriculum_stages - 1)