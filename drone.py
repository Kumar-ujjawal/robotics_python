import pybullet as p
import pybullet_data
import time
import numpy as np

# Initialize PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

# Load drone model (using a simple box for this example)
drone = p.loadURDF("cube.urdf", [0, 0, 1], globalScaling=0.2)

# PID controller parameters
Kp_altitude = 20
Ki_altitude = 0.1
Kd_altitude = 10

Kp_position = 5
Ki_position = 0.1
Kd_position = 2

Kp_orientation = 10
Ki_orientation = 0.1
Kd_orientation = 5

# Target altitude
target_altitude = 2.0

# PID variables
integral_altitude = 0
prev_error_altitude = 0
integral_position = np.zeros(2)
prev_error_position = np.zeros(2)
integral_orientation = np.zeros(3)
prev_error_orientation = np.zeros(3)

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
camera_distance = 0.5
fov = 60
aspect = 1.0
near = 0.1
far = 100

# Minimum time step to prevent division by zero
MIN_TIME_STEP = 1e-6

def move_to_point(current_pos, target_pos, dt):
    global integral_position, prev_error_position
    error = np.array(target_pos) - np.array(current_pos[:2])
    integral_position += error * dt
    if dt > MIN_TIME_STEP:
        derivative = (error - prev_error_position) / dt
    else:
        derivative = np.zeros_like(error)
    output = Kp_position * error + Ki_position * integral_position + Kd_position * derivative
    prev_error_position = error
    return output

def stabilize_orientation(current_orn, dt):
    global integral_orientation, prev_error_orientation
    current_orn = np.array(p.getEulerFromQuaternion(current_orn))
    target_orn = np.array([0, 0, current_orn[2]])  # We want to keep the yaw, but zero out roll and pitch
    error = target_orn - current_orn
    integral_orientation += error * dt
    if dt > MIN_TIME_STEP:
        derivative = (error - prev_error_orientation) / dt
    else:
        derivative = np.zeros_like(error)
    output = Kp_orientation * error + Ki_orientation * integral_orientation + Kd_orientation * derivative
    prev_error_orientation = error
    return output

def capture_image():
    drone_pos, drone_orn = p.getBasePositionAndOrientation(drone)
    rotation_matrix = p.getMatrixFromQuaternion(drone_orn)
    rotation_matrix = np.array(rotation_matrix).reshape(3, 3)
    camera_vector = np.array([0, 0, -camera_distance])
    camera_vector = rotation_matrix.dot(camera_vector)
    camera_position = np.array(drone_pos) + camera_vector
    
    view_matrix = p.computeViewMatrix(
        cameraEyePosition=camera_position,
        cameraTargetPosition=drone_pos,
        cameraUpVector=rotation_matrix[:, 1]
    )
    projection_matrix = p.computeProjectionMatrixFOV(
        fov=fov, aspect=aspect, nearVal=near, farVal=far
    )
    
    width, height, rgbImg, depthImg, segImg = p.getCameraImage(
        width=320, height=240, viewMatrix=view_matrix, projectionMatrix=projection_matrix
    )
    print(f"Camera image captured at position {drone_pos}")

# Simulation loop
try:
    last_time = time.time()
    while True:
        current_time = time.time()
        dt = max(current_time - last_time, MIN_TIME_STEP)  # Ensure dt is never zero
        last_time = current_time

        # Get drone position and orientation
        pos, orn = p.getBasePositionAndOrientation(drone)
        
        # Altitude control
        error_altitude = target_altitude - pos[2]
        integral_altitude += error_altitude * dt
        derivative_altitude = (error_altitude - prev_error_altitude) / dt
        altitude_output = Kp_altitude * error_altitude + Ki_altitude * integral_altitude + Kd_altitude * derivative_altitude
        prev_error_altitude = error_altitude
        
        # Horizontal movement
        current_target = rectangle_points[current_point_index]
        horizontal_force = move_to_point(pos, current_target, dt)
        
        # Orientation stabilization
        orientation_torque = stabilize_orientation(orn, dt)
        
        # Combine forces and torques
        total_force = [horizontal_force[0], horizontal_force[1], altitude_output]
        
        # Apply forces and torques
        p.applyExternalForce(drone, -1, total_force, [0, 0, 0], p.WORLD_FRAME)
        p.applyExternalTorque(drone, -1, orientation_torque, p.WORLD_FRAME)
        
        # Check if we've reached the current target point
        distance_to_target = np.linalg.norm(np.array(current_target) - np.array(pos[:2]))
        if distance_to_target < point_threshold:
            current_point_index = (current_point_index + 1) % len(rectangle_points)
        
        # Capture image
        capture_image()
        
        # Step the simulation
        p.stepSimulation()
        
        time.sleep(1./240.)  # Simulate at 240 Hz

except KeyboardInterrupt:
    print("Simulation stopped by user")

finally:
    # Disconnect from PyBullet
    p.disconnect()