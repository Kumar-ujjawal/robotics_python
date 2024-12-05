import pybullet as p
import pybullet_data
import time
import math
import numpy as np

# Connect to the PyBullet physics engine
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Set the path to the Kinova robot arm URDF file
urdf_path = r"kinova-ros\kinova_description\urdf\j2s7s300_standalone.urdf"

# Load the plane and robot arm
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=True)

# Get the joint indices for controllable joints
num_joints = p.getNumJoints(robotId)
controllable_joints = []
for i in range(num_joints):
    joint_info = p.getJointInfo(robotId, i)
    if joint_info[2] != p.JOINT_FIXED:
        controllable_joints.append(i)

print(f"Number of controllable joints: {len(controllable_joints)}")

# Function to get the end effector position
def get_end_effector_pos():
    state = p.getLinkState(robotId, num_joints - 1)
    return state[0]

# Function to perform inverse kinematics
def inverse_kinematics(target_pos):
    joint_poses = p.calculateInverseKinematics(robotId, num_joints - 1, target_pos)
    return joint_poses[:len(controllable_joints)]

# Function to move the robot arm to a target position
def move_to_target(start_pos, end_pos, trajectory_time):
    steps = int(240 * trajectory_time)  # 240 Hz for the given time
    for t in range(steps):
        # Calculate the current target position along the straight line
        alpha = t / steps
        current_target = [
            start_pos[0] + alpha * (end_pos[0] - start_pos[0]),
            start_pos[1] + alpha * (end_pos[1] - start_pos[1]),
            start_pos[2] + alpha * (end_pos[2] - start_pos[2])
        ]

        # Perform inverse kinematics
        joint_poses = inverse_kinematics(current_target)

        # Set the joint positions
        for i, joint in enumerate(controllable_joints):
            p.setJointMotorControl2(
                bodyUniqueId=robotId,
                jointIndex=joint,
                controlMode=p.POSITION_CONTROL,
                targetPosition=joint_poses[i],
                force=100.0
            )

        p.stepSimulation()
        time.sleep(1.0 / 240.0)

# Main loop to handle multiple end positions
def main():
     
    waypoints = [
        [0.5337958105113186, -0.10650478491209342, 0.4985054933668144],
        [0.5337958105113186, -0.10650478491209342, 0.4985054933668144],
        [-0.0071203869026121025, 0.32445829567672363, 0.37930713793632564],
        [0.29439394759070736, 0.4104775261153995, 0.8633216574635938]
    ]

    # Time to spend moving between each waypoint
    trajectory_time = 3.0  # seconds

    start_pos = get_end_effector_pos()

    for waypoint in waypoints:
        print(f"Moving to waypoint: {waypoint}")
        move_to_target(start_pos, waypoint, trajectory_time)
        start_pos = waypoint  # Update start position for the next movement

    print("Finished moving through all waypoints")

    # Keep the GUI window open and maintain the final position
    while True:
        p.stepSimulation()
        time.sleep(1.0 / 240.0)

if __name__ == "__main__":
    main()