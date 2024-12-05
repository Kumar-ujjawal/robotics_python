import pybullet as p
import pybullet_data
import time
import math
import numpy as np

# Initialize PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

# Load drone model (using a simple box for this example)
drone = p.loadURDF("cube.urdf", [0, 0, 1], globalScaling=0.2)

# PID controller parameters
Kp = 10
Ki = 0.1
Kd = 5

# Target altitude
target_altitude = 1.5

# PID variables
integral = 0
prev_error = 0

# Rectangular path parameters
rectangle_points = [
    (0, 0),
    (2, 0),
    (2, 2),
    (0, 2)
]
current_point_index = 0
point_threshold = 0.1

# Camera parameters
camera_distance = 0.5  # Distance from drone center to camera
fov = 60
aspect = 1.0
near = 0.1
far = 100

def move_to_point(current_pos, target_pos):
    direction = np.array(target_pos) - np.array(current_pos[:2])
    distance = np.linalg.norm(direction)
    if distance > 0:
        direction = direction / distance
    return direction * 5  # Adjust this multiplier to change speed

def capture_image():
    drone_pos, drone_orn = p.getBasePositionAndOrientation(drone)
    rotation_matrix = p.getMatrixFromQuaternion(drone_orn)
    camera_vector = [0, 0, -camera_distance]
    camera_vector = np.dot(rotation_matrix, camera_vector)
    camera_position = drone_pos + camera_vector
    
    view_matrix = p.computeViewMatrix(
        cameraEyePosition=camera_position,
        cameraTargetPosition=drone_pos,
        cameraUpVector=[0, 1, 0]
    )
    projection_matrix = p.computeProjectionMatrixFOV(
        fov=fov,
        aspect=aspect,
        nearVal=near,
        farVal=far
    )
    
    width, height, rgbImg, depthImg, segImg = p.getCameraImage(
        width=320,
        height=240,
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix
    )
    print(f"Camera image captured at position {drone_pos}")

# Simulation loop
while True:
    # Get drone position and orientation
    pos, orn = p.getBasePositionAndOrientation(drone)
    
    # Altitude control
    error = target_altitude - pos[2]
    integral += error
    derivative = error - prev_error
    altitude_output = Kp * error + Ki * integral + Kd * derivative
    
    # Horizontal movement
    current_target = rectangle_points[current_point_index]
    horizontal_force = move_to_point(pos, current_target)
    
    # Apply forces
    p.applyExternalForce(drone, -1, [horizontal_force[0], horizontal_force[1], altitude_output], [0, 0, 0], p.WORLD_FRAME)
    
    # Check if we've reached the current target point
    distance_to_target = np.linalg.norm(np.array(current_target) - np.array(pos[:2]))
    if distance_to_target < point_threshold:
        current_point_index = (current_point_index + 1) % len(rectangle_points)
    
    # Capture image
    capture_image()
    
    # Step the simulation
    p.stepSimulation()
    
    # Update previous error
    prev_error = error
    
    time.sleep(1./240.)  # Simulate at 240 Hz

# Disconnect from PyBullet (this line will not be reached in this example)
p.disconnect()