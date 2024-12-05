import pybullet as p
import pybullet_data
import cv2
import time
import os
import numpy as np
import google.generativeai as genai
from api import api_keys

# Configure API Key for Google Generative AI (optional, for LLM response)
ak = api_keys()
api_key_1 = ak.get_private_vars()
os.environ['GOOGLE_API_KEY'] = api_key_1
genai.configure(api_key=api_key_1)
model = genai.GenerativeModel(model_name="gemini-1.5-flash")

# Connect to PyBullet physics engine
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Load environment
planeId = p.loadURDF("plane.urdf")
table_position = [0.5, 0, 0]
tableId = p.loadURDF("table/table.urdf", table_position, [0, 0, 0, 1], useFixedBase=True)

# Load robot arm
robot_position = [0.2, -0.4, 0.9]
urdf_path = r"kinova-ros\kinova_description\urdf\j2s7s300_standalone.urdf"  # Update with your URDF path
robotId = p.loadURDF(urdf_path, robot_position, useFixedBase=True)
num_joints = p.getNumJoints(robotId)
controllable_joints = []
for i in range(num_joints):
    joint_info = p.getJointInfo(robotId, i)
    if joint_info[2] != p.JOINT_FIXED:
        controllable_joints.append(i)
# Initialize objectId for spawning control
objectId = None

# Detect bottle using OpenCV
def detect_bottle(video_feed):
    """Detect a bottle using the camera feed."""
    ret, frame = video_feed.read()
    if not ret:
        return False

    # Convert to grayscale
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Thresholding for bottle-like shapes (modify as per requirements)
    _, threshold = cv2.threshold(gray_frame, 127, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Check for bottle-like contours
    for contour in contours:
        area = cv2.contourArea(contour)
        if 5000 < area < 20000:  # Area range for bottle shape
            return True
    return False

# Spawn object in PyBullet
def spawn_object_on_signal(object_position, object_orientation):
    """Spawn an object on the table if signal is received."""
    global objectId
    if objectId is None:  # Avoid multiple spawns
        objectId = p.loadURDF("cube_small.urdf", object_position, object_orientation, useFixedBase=False)
        print("Object spawned on the table.")
# pick up

def close_gripper():
    """Simulate closing of the gripper."""
    print("Gripper closing...")
   

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
# Handle LLM response (optional)
def get_llm_response(user_input):
    """Send user input to LLM and get a response."""
    prompt = f"Respond to the command: '{user_input}'. If it involves picking an object, confirm the action. if object is not present wait for object avaialbility, and then confirm the action"
    response = model.generate_content(prompt)
    return response.text
def detect_object():
    """Simulate camera detection of the object and return its position."""
    object_state = p.getBasePositionAndOrientation(objectId)
    return object_state[0]  # Return only the position
# Main function
def main():
    print("Welcome to the Robot Arm Control System with Bottle Detection!")
    video_feed = cv2.VideoCapture(0)  # OpenCV video feed (camera index 0)

    # Main loop
    while True:
        user_input = input("Enter a command (or type 'exit' to quit): ")

        if user_input.lower() == 'exit':
            break

        # Detect bottle
        bottle_detected = detect_bottle(video_feed)
        if bottle_detected:
            print("Bottle detected! Spawning object on the table.")
            spawn_object_on_signal(object_position=[0.6, 0, 0.64], object_orientation=p.getQuaternionFromEuler([0, 0, 0]))
            llm_response = get_llm_response(user_input)
            if 'pick' in user_input.lower():
                object_coords = detect_object()
                print(f"Object detected at: {object_coords}")

                pick_object(object_coords)
                print("Object picked successfully.")
        else:
            print("No bottle detected in the frame.")
            llm_response = get_llm_response("no object availble on table, so dont actuate joints")

        # Optional: Interact with the LLM
        
        print("LLM Response:", llm_response)

    # Cleanup
    video_feed.release()
    cv2.destroyAllWindows()
    p.disconnect()

# Run the program
if __name__ == "__main__":
    main()
