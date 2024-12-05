import pybullet as p
import pybullet_data
import time
import numpy as np
import os
import google.generativeai as genai
from api import api_keys
# model_a = r"models\actor.pth"
# model_c = r"models\critic.pth"
# Configure API Key
ak = api_keys()
api_key_1 = ak.get_private_vars()

os.environ['GOOGLE_API_KEY'] = api_key_1
genai.configure(api_key=api_key_1)

model = genai.GenerativeModel(model_name="gemini-1.5-flash")

# Connect to physics engine
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Load Table
planeId = p.loadURDF("plane.urdf")
table_position = [0.5, 0, 0]
tableId = p.loadURDF("table/table.urdf", table_position, [0, 0, 0, 1], useFixedBase=True)

# Load Robot Arm
robot_position = [0.2, -0.4, 0.9]
urdf_path = r"kinova-ros\kinova_description\urdf\j2s7s300_standalone.urdf"  # Provide your URDF file path here
robotId = p.loadURDF(urdf_path, robot_position, useFixedBase=True)

# Load Object on Table
object_position = [0.6, 0, 0.64]
object_orientation = p.getQuaternionFromEuler([0, 0, 0])
objectId = p.loadURDF("cube_small.urdf", object_position, object_orientation, useFixedBase=False)

# Get controllable joints
num_joints = p.getNumJoints(robotId)
controllable_joints = []
for i in range(num_joints):
    joint_info = p.getJointInfo(robotId, i)
    if joint_info[2] != p.JOINT_FIXED:
        controllable_joints.append(i)

print(f"Number of controllable joints: {len(controllable_joints)}")

# Functions
def get_end_effector_pos():
    state = p.getLinkState(robotId, num_joints - 1)
    return state[0]

def inverse_kinematics(target_pos):
    joint_poses = p.calculateInverseKinematics(robotId, num_joints - 1, target_pos)
    return joint_poses[:len(controllable_joints)]

def move_to_target(start_pos, end_pos, trajectory_time):
    steps = int(240 * trajectory_time)  # 240 Hz for the given time
    for t in range(steps):
        alpha = t / steps
        current_target = [
            start_pos[0] + alpha * (end_pos[0] - start_pos[0]),
            start_pos[1] + alpha * (end_pos[1] - start_pos[1]),
            start_pos[2] + alpha * (end_pos[2] - start_pos[2])
        ]
        joint_poses = inverse_kinematics(current_target)
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

def detect_object():
    """Simulate camera detection of the object and return its position."""
    object_state = p.getBasePositionAndOrientation(objectId)
    return object_state[0]  # Return only the position

def get_llm_response(user_input):
    """Send user input to LLM and get a response."""
    prompt = f"Respond to the command: '{user_input}'. If it involves picking an object, confirm the action."
    response = model.generate_content(prompt)
    return response.text

def close_gripper():
    """Simulate closing of the gripper."""
    print("Gripper closing...")
    # Add your gripper-specific code here (e.g., joint movements).

def open_gripper():
    """Simulate opening of the gripper."""
    print("Gripper opening...")
    # Add your gripper-specific code here (e.g., joint movements).

def pick_object(object_pos):
    """Move the robotic arm to the object's position, pick it, and simulate a pick action."""
    # Move above the object
    hover_pos = [object_pos[0], object_pos[1], object_pos[2] + 0.1]
    move_to_target(get_end_effector_pos(), hover_pos, trajectory_time=3.0)

    # Move to the object's position
    move_to_target(hover_pos, object_pos, trajectory_time=2.0)

    # Close the gripper to pick the object
    close_gripper()

    # Move back to hover position
    move_to_target(object_pos, hover_pos, trajectory_time=2.0)

def place_object(place_pos):
    """Move the robotic arm to a new location and place the object."""
    # Move above the placement position
    hover_pos = [place_pos[0], place_pos[1], place_pos[2] + 0.1]
    move_to_target(get_end_effector_pos(), hover_pos, trajectory_time=3.0)

    # Move to the placement position
    move_to_target(hover_pos, place_pos, trajectory_time=2.0)

    # Open the gripper to release the object
    open_gripper()

    # Move back to hover position
    move_to_target(place_pos, hover_pos, trajectory_time=2.0)

def main():
    print("Welcome to the Enhanced Robot Arm Control System!")
    while True:
        user_input = input("Enter your command (e.g., 'Pick and place the object') or type 'exit' to quit: ")

        if user_input.lower() == 'exit':
            break

        llm_response = get_llm_response(user_input)
        print("LLM Response:", llm_response)

        if 'pick' in user_input.lower():
            object_coords = detect_object()
            print(f"Object detected at: {object_coords}")

            pick_object(object_coords)
            print("Object picked successfully.")

            # Define a new position for placing the object
            place_position = [0.4, -0.2, 0.64]
            print(f"Placing object at: {place_position}")

            place_object(place_position)
            print("Object placed successfully.")

    p.disconnect()

if __name__ == "__main__":
    main()
