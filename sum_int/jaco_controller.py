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
initial_joint_positions = [0.0, 2.9, 1.3, -2.07, 1.4, 0.0, 0.0, 1.0, 1.0, 1.0]
robotId = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=True)

# Set initial joint positions
for i, pos in enumerate(initial_joint_positions):
    p.resetJointState(robotId, i, pos)

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

# Create sliders for joint control
sliders = []
for i, joint in enumerate(controllable_joints):
    joint_info = p.getJointInfo(robotId, joint)
    lower_limit = joint_info[8]
    upper_limit = joint_info[9]
    
    # If the joint is continuous (no limits), set arbitrary limits for the slider
    if math.isclose(lower_limit, upper_limit, abs_tol=1e-5):
        lower_limit = -math.pi
        upper_limit = math.pi
    
    initial_value = p.getJointState(robotId, joint)[0]
    slider = p.addUserDebugParameter(f"{joint_names[i]}", lower_limit, upper_limit, initial_value)
    sliders.append(slider)

# Function to get joint positions and velocities
def get_joint_states():
    joint_states = p.getJointStates(robotId, controllable_joints)
    positions = [state[0] for state in joint_states]
    velocities = [state[1] for state in joint_states]
    return positions, velocities

# Function to get the end effector position
def get_end_effector_pos():
    state = p.getLinkState(robotId, num_joints - 1)
    return state[0]

# Main simulation loop
try:
    while True:
        # Get slider values and set joint positions
        for i, slider in enumerate(sliders):
            target_pos = p.readUserDebugParameter(slider)
            p.setJointMotorControl2(
                bodyUniqueId=robotId,
                jointIndex=controllable_joints[i],
                controlMode=p.POSITION_CONTROL,
                targetPosition=target_pos,
                force=500.0,  # Increased force for better responsiveness
                maxVelocity=1.0  # Limit velocity to prevent too rapid movement
            )

        # Step the simulation
        p.stepSimulation()

        # Get and print joint states
        positions, velocities = get_joint_states()
        end_effector_pos = get_end_effector_pos()

        print("Joint Positions:", positions)
        print("Joint Velocities:", velocities)
        print("End Effector Position:", end_effector_pos)
        print("--------------------")

        time.sleep(1.0 / 240.0)

except KeyboardInterrupt:
    p.disconnect()
    print("Simulation ended by user")