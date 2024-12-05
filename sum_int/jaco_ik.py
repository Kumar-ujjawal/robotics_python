import pybullet as p
import pybullet_data
import time
import numpy as np
from scipy.optimize import least_squares

class KinovaJacoArm:
    def __init__(self, urdf_path):
        self.urdf_path = urdf_path
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        self.planeId = p.loadURDF("plane.urdf")
        self.robotId = p.loadURDF(self.urdf_path, [0, 0, 0], useFixedBase=True)
        
        self.jointIndices = self.get_controllable_joints()
        self.num_joints = len(self.jointIndices)
        
        self.joint_lower_limits, self.joint_upper_limits, self.joint_velocity_limits = self.get_joint_limits()

    def get_controllable_joints(self):
        jointIndices = []
        for i in range(p.getNumJoints(self.robotId)):
            jointInfo = p.getJointInfo(self.robotId, i)
            if jointInfo[2] != p.JOINT_FIXED:
                jointIndices.append(i)
        return jointIndices

    def get_joint_limits(self):
        joint_lower_limits = []
        joint_upper_limits = []
        joint_velocity_limits = []
        for i in self.jointIndices:
            info = p.getJointInfo(self.robotId, i)
            joint_lower_limits.append(info[8])
            joint_upper_limits.append(info[9])
            joint_velocity_limits.append(info[11])
        return joint_lower_limits, joint_upper_limits, joint_velocity_limits

    def set_joint_positions(self, joint_positions):
        p.setJointMotorControlArray(
            bodyUniqueId=self.robotId,
            jointIndices=self.jointIndices,
            controlMode=p.POSITION_CONTROL,
            targetPositions=joint_positions,
            forces=[100.0] * self.num_joints,
        )

    def get_joint_states(self):
        joint_states = p.getJointStates(self.robotId, self.jointIndices)
        return [state[0] for state in joint_states]

    def get_end_effector_pose(self):
        state = p.getLinkState(self.robotId, self.jointIndices[-1])
        return state[0] + state[1]  # position + orientation (quaternion)

    def objective_function(self, joint_angles, target_pose):
        # Set the joint angles
        self.set_joint_positions(joint_angles)
        p.stepSimulation()

        # Get the current end-effector pose
        current_pose = self.get_end_effector_pose()
        
        # Compute the error
        position_error = np.linalg.norm(np.array(current_pose[:3]) - np.array(target_pose[:3]))
        orientation_error = np.linalg.norm(np.array(current_pose[3:]) - np.array(p.getQuaternionFromEuler(target_pose[3:])))
        
        return position_error + orientation_error

    def inverse_kinematics(self, target_pose, initial_guess=None):
        if initial_guess is None:
            initial_guess = self.get_joint_states()
        
        result = least_squares(
            self.objective_function,
            initial_guess,
            args=(target_pose,),
            bounds=(self.joint_lower_limits, self.joint_upper_limits)
        )
        
        return result.x

    def move_to_pose(self, target_pose, steps=100):
        current_joint_angles = self.get_joint_states()
        target_joint_angles = self.inverse_kinematics(target_pose, current_joint_angles)
        
        for i in range(steps):
            intermediate_angles = current_joint_angles + (target_joint_angles - current_joint_angles) * (i + 1) / steps
            self.set_joint_positions(intermediate_angles)
            p.stepSimulation()
            time.sleep(1.0 / 240.0)

    def run_simulation(self):
        while True:
            p.stepSimulation()
            time.sleep(1.0 / 240.0)

if __name__ == "__main__":
    urdf_path = r"kinova-ros\kinova_description\urdf\j2s7s300_standalone.urdf"
    arm = KinovaJacoArm(urdf_path)
    
    # Example usage:
    target_pose = [0.2, 0.2, 0.5, 0, 0, 0]  # x, y, z, roll, pitch, yaw
    arm.move_to_pose(target_pose)
    
    arm.run_simulation()