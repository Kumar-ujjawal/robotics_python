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

# Load the plane
planeId = p.loadURDF("plane.urdf")

# Load the robot arm with specified initial joint positions
initial_joint_positions = [0.0, 0.0, 2.9, 1.3, -2.07, 1.4, 0.0, 1.0, 1.0, 1.0]
robotId = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=True)

# Set initial joint positions
for i, pos in enumerate(initial_joint_positions):
    p.resetJointState(robotId, i, pos)

# Unlock all joints
for i in range(p.getNumJoints(robotId)):
    p.setJointMotorControl2(robotId, i, p.VELOCITY_CONTROL, force=0)

# Get the joint indices for controllable joints
num_joints = p.getNumJoints(robotId)
controllable_joints = []
joint_names = []
for i in range(num_joints):
    joint_info = p.getJointInfo(robotId, i)
    if joint_info[2] != p.JOINT_FIXED:
        controllable_joints.append(i)
        joint_names.append(joint_info[1].decode('utf-8'))
    print(f"Joint {i}: {joint_info[1].decode('utf-8')}, Type: {joint_info[2]}, Lower Limit: {joint_info[8]}, Upper Limit: {joint_info[9]}")

print(f"Number of controllable joints: {len(controllable_joints)}")
print(f"Active joint names: {joint_names}")

# Create sliders for joint control
sliders = []
for i, joint in enumerate(controllable_joints):
    joint_info = p.getJointInfo(robotId, joint)
    lower_limit = joint_info[8]
    upper_limit = joint_info[9]
    
    # If lower and upper limits are equal, treat as continuous joint
    if math.isclose(lower_limit, upper_limit):
        lower_limit = -math.pi
        upper_limit = math.pi
    
    initial_value = p.getJointState(robotId, joint)[0]
    slider = p.addUserDebugParameter(f"{joint_names[i]}", lower_limit, upper_limit, initial_value)
    sliders.append(slider)

# Main simulation loop
try:
    while True:
        # Get slider values and set joint positions
        for i, slider in enumerate(sliders):
            target_pos = p.readUserDebugParameter(slider)
            joint_info = p.getJointInfo(robotId, controllable_joints[i])
            
            if math.isclose(joint_info[8], joint_info[9]):  # Continuous joint
                current_pos = p.getJointState(robotId, controllable_joints[i])[0]
                error = target_pos - current_pos
                # Normalize the error to be between -pi and pi
                error = (error + math.pi) % (2 * math.pi) - math.pi
                p.setJointMotorControl2(
                    bodyUniqueId=robotId,
                    jointIndex=controllable_joints[i],
                    controlMode=p.VELOCITY_CONTROL,
                    targetVelocity=error * 5.0,  # Adjust this multiplier as needed
                    force=1000.0
                )
            else:
                # For other joints, use position control
                p.setJointMotorControl2(
                    bodyUniqueId=robotId,
                    jointIndex=controllable_joints[i],
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=target_pos,
                    force=1000.0,
                    maxVelocity=1.0
                )

        # Step the simulation
        p.stepSimulation()

        # Get and print joint states
        joint_states = p.getJointStates(robotId, controllable_joints)
        positions = [state[0] for state in joint_states]
        velocities = [state[1] for state in joint_states]
        end_effector_pos = p.getLinkState(robotId, num_joints - 1)[0]

        print("Joint Positions:", positions)
        print("Joint Velocities:", velocities)
        print("End Effector Position:", end_effector_pos)
        print("--------------------")

        time.sleep(1.0 / 240.0)

except KeyboardInterrupt:
    p.disconnect()
    print("Simulation ended by user")