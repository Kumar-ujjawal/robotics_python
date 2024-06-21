import pybullet as p
import pybullet_data
import time
import math
import os

# Connect to the PyBullet physics engine
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Set the path to the Kinova robot arm URDF file
urdf_path = r"kinova-ros\kinova_description\urdf\j2s7s300_standalone.urdf"
print(f"URDF path: {urdf_path}")

# Load the plane and robot arm
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF(urdf_path, [0, 0, 0.5], useFixedBase=True)

# Get the joint indices for controllable joints
jointIndices = []
for i in range(p.getNumJoints(robotId)):
    jointInfo = p.getJointInfo(robotId, i)
    if jointInfo[2] != p.JOINT_FIXED:
        jointIndices.append(i)

print(f"Number of controllable joints: {len(jointIndices)}")

# Set the initial joint positions
initialJointPositions = [0.0] * len(jointIndices)
p.setJointMotorControlArray(
    bodyUniqueId=robotId,
    jointIndices=jointIndices,
    controlMode=p.POSITION_CONTROL,
    targetPositions=initialJointPositions,
    forces=[100.0] * len(jointIndices),
)

# Define the target joint positions for a specific configuration
# Make sure this matches the number of controllable joints
targetJointPositions = [0.0] * len(jointIndices)

# Simulate the robot arm movement
for t in range(1000):
    p.stepSimulation()
    p.setJointMotorControlArray(
        bodyUniqueId=robotId,
        jointIndices=jointIndices,
        controlMode=p.POSITION_CONTROL,
        targetPositions=targetJointPositions,
        forces=[100.0] * len(jointIndices),
    )
    time.sleep(1.0 / 240.0)

# Keep the GUI window open
while True:
    p.stepSimulation()
    time.sleep(1.0 / 240.0)