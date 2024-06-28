import pybullet as p
import pybullet_data
import numpy as np
import math

class KinovaEnv:
    def __init__(self):
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        urdf_path = "kinova-ros/kinova_description/urdf/j2s7s300_standalone.urdf"
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
        self.distance_threshold = 0.001  # Threshold to consider as "no progress"
        self.max_steps_without_progress = 10  # Maximum steps allowed without progress

    def reset(self):
        for i, pos in enumerate(self.initial_joint_positions):
            p.resetJointState(self.robotId, i, pos)
        p.resetBasePositionAndOrientation(self.ballId, self.ball_position, [0, 0, 0, 1])
        self.last_distance = self._get_distance_to_ball()
        self.steps_without_progress = 0
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
        
        for _ in range(10):  # Increase simulation steps per action
            p.stepSimulation()

        state = self._get_state()
        reward = self._compute_reward()
        done = self._is_done()

        self.last_distance = current_distance

        return state, reward, done

    def _take_random_action(self):
        return np.random.uniform(-1, 1, self.action_space)

    def _get_state(self):
        joint_states = p.getJointStates(self.robotId, self.controllable_joints)
        joint_positions = np.array([state[0] for state in joint_states])
        joint_velocities = np.array([state[1] for state in joint_states])
        end_effector_pos = np.array(p.getLinkState(self.robotId, len(self.controllable_joints) - 1)[0])
        return np.concatenate([joint_positions, joint_velocities, end_effector_pos])

    def _get_distance_to_ball(self):
        end_effector_pos = np.array(p.getLinkState(self.robotId, len(self.controllable_joints) - 1)[0])
        return np.linalg.norm(end_effector_pos - self.ball_position)

    def _compute_reward(self):
        current_distance = self._get_distance_to_ball()
        
        if self.last_distance is None:
            reward = 0
        else:
            reward = self.last_distance - current_distance
        
        if current_distance < 0.1:
            reward += 10
        
        if current_distance > 1.0:
            reward -= 5
        
        return reward

    def _is_done(self):
        return self._get_distance_to_ball() < 0.1

    def close(self):
        p.disconnect(self.physicsClient)