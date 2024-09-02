import pybullet as p
import pybullet_data
import time
import math
import random
import cv2
import numpy as np

# Connect to the PyBullet physics engine
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Load the plane
planeId = p.loadURDF("plane.urdf")

# Create conveyor belt
conveyor_length = 5
conveyor_width = 0.5
conveyor_height = 0.2
conveyor_position = [0, 0, conveyor_height/2]

conveyor_visual = p.createVisualShape(shapeType=p.GEOM_BOX, 
                                      halfExtents=[conveyor_length/2, conveyor_width/2, conveyor_height/2],
                                      rgbaColor=[0.5, 0.5, 0.5, 1])
conveyor_collision = p.createCollisionShape(shapeType=p.GEOM_BOX, 
                                            halfExtents=[conveyor_length/2, conveyor_width/2, conveyor_height/2])
conveyor_body = p.createMultiBody(baseMass=0,
                                  baseCollisionShapeIndex=conveyor_collision,
                                  baseVisualShapeIndex=conveyor_visual,
                                  basePosition=conveyor_position)

# Load the Kinova robot arm
urdf_path = r"kinova-ros\kinova_description\urdf\j2s7s300_standalone.urdf"
robotId = p.loadURDF(urdf_path, [conveyor_length/2, -conveyor_width, conveyor_height], useFixedBase=True)

# Get the joint indices for controllable joints
jointIndices = [j for j in range(p.getNumJoints(robotId)) if p.getJointInfo(robotId, j)[2] != p.JOINT_FIXED]

# Set initial joint positions
initialJointPositions = [0.0] * len(jointIndices)
p.setJointMotorControlArray(robotId, jointIndices, p.POSITION_CONTROL, targetPositions=initialJointPositions)

# Create product models (simplified as colored boxes)
def create_product(position):
    size = [0.1, 0.05, 0.2]
    visual = p.createVisualShape(shapeType=p.GEOM_BOX, 
                                 halfExtents=size,
                                 rgbaColor=[random.random(), random.random(), random.random(), 1])
    collision = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=size)
    body = p.createMultiBody(baseMass=0.1,
                             baseCollisionShapeIndex=collision,
                             baseVisualShapeIndex=visual,
                             basePosition=position)
    return body

# Function to move products on the conveyor
def move_products(products, speed):
    for product in products:
        pos, _ = p.getBasePositionAndOrientation(product)
        new_pos = [pos[0] - speed, pos[1], pos[2]]
        p.resetBasePositionAndOrientation(product, new_pos, [0, 0, 0, 1])

# Function to check if a product is defective (simplified)
def is_defective(product):
    return random.random() < 0.2  # 20% chance of being defective

# Function to get camera image
def get_camera_image():
    width, height, rgbImg, depthImg, segImg = p.getCameraImage(
        width=320, height=240,
        viewMatrix=p.computeViewMatrix(
            cameraEyePosition=[0, -2, 2],
            cameraTargetPosition=[0, 0, 0],
            cameraUpVector=[0, 0, 1]
        ),
        projectionMatrix=p.computeProjectionMatrixFOV(
            fov=60, aspect=1.33, nearVal=0.1, farVal=100
        )
    )
    return cv2.cvtColor(rgbImg, cv2.COLOR_RGBA2RGB)

# Main simulation loop
products = []
conveyor_speed = 0.01
next_spawn_time = 0
spawn_interval = 2  # seconds

for t in range(10000):
    p.stepSimulation()
    
    # Spawn new products
    if time.time() > next_spawn_time:
        product = create_product([conveyor_length/2, 0, conveyor_height + 0.1])
        products.append(product)
        next_spawn_time = time.time() + spawn_interval
    
    # Move products on the conveyor
    move_products(products, conveyor_speed)
    
    # Process products at the end of the conveyor
    for product in products:
        pos, _ = p.getBasePositionAndOrientation(product)
        if pos[0] < -conveyor_length/2:
            if is_defective(product):
                # Move defective product off the conveyor
                p.resetBasePositionAndOrientation(product, [pos[0], conveyor_width, pos[2]], [0, 0, 0, 1])
            products.remove(product)
            p.removeBody(product)
    
    # Capture and process camera image (simplified)
    if t % 100 == 0:  # Process every 100 steps
        image = get_camera_image()
        # Here you would typically do more complex image processing
        # implement  box detection logic here and extract coordinate of the box
        # send arm to grab the yellow colored boxed from the ocnver belt
        # For demonstration, we'll just show the image
        cv2.imshow('Camera View', image)
        cv2.waitKey(1)
    
    time.sleep(1./240.)

# Clean up
cv2.destroyAllWindows()
p.disconnect()