import pybullet as p
import pybullet_data
import time
import math
import numpy as np
import os
import google.generativeai as genai
from api import api_keys


ak = api_keys()
api_key_1 = ak.get_private_vars()

os.environ['GOOGLE_API_KEY'] = api_key_1
genai.configure(api_key=api_key_1)

model = genai.GenerativeModel(model_name="gemini-1.5-flash")

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)


urdf_path = r"kinova-ros\kinova_description\urdf\j2s7s300_standalone.urdf"      #for user, your urdf file here 

planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=True)

num_joints = p.getNumJoints(robotId)
controllable_joints = []
for i in range(num_joints):
    joint_info = p.getJointInfo(robotId, i)
    if joint_info[2] != p.JOINT_FIXED:
        controllable_joints.append(i)

print(f"Number of controllable joints: {len(controllable_joints)}")

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

def get_llm_waypoints(user_input):
    prompt = f"Generate a list of 3D coordinates (x, y, z) for waypoints that would make a robot arm follow a trajectory or shape described as: '{user_input}'. Please provide at least 4 waypoints, with each waypoint on a new line in the format [x, y, z]. Ensure the coordinates are within the robot's reach, typically within a range of -1 to 1 for each axis."
    
    response = model.generate_content(prompt)
    return response.text

def parse_waypoints(llm_response):
    waypoints = []
    lines = llm_response.split('\n')
    for line in lines:
        if line.startswith('[') and line.endswith(']'):
            try:
                coords = eval(line)
                if isinstance(coords, list) and len(coords) == 3:
                    waypoints.append(coords)
            except:
                pass
    return waypoints

def main():
    print("Welcome to the LLM-Guided Robot Arm Control System!")
    while True:
        user_input = input("Please describe the trajectory or shape you want the robot arm to follow (or type 'exit' to quit): ")
        
        if user_input.lower() == 'exit':
            break
        
        llm_response = get_llm_waypoints(user_input)
        print("LLM Response:", llm_response)
        
        waypoints = parse_waypoints(llm_response)
        
        if not waypoints:
            print("Unable to generate valid waypoints. Please try again with a different description.")
            continue
        
        print("Generated Waypoints:", waypoints)
        
        start_pos = get_end_effector_pos()
        for waypoint in waypoints:
            print(f"Moving to waypoint: {waypoint}")
            move_to_target(start_pos, waypoint, trajectory_time=3.0)
            start_pos = waypoint
        
        print("Finished moving through all waypoints")

    p.disconnect()

if __name__ == "__main__":
    main()