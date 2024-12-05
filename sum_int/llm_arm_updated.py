import pybullet as p
import pybullet_data
import numpy as np
import time
import cv2
import torch
import torch.nn as nn

class DepthSensorSimulator:
    def __init__(self, robot_id, camera_link_index):
        """
        Initialize depth sensor simulation in PyBullet
        
        Args:
            robot_id (int): PyBullet robot unique ID
            camera_link_index (int): Link index for camera attachment
        """
        self.robot_id = robot_id
        self.camera_link_index = camera_link_index
        
        # Camera parameters
        self.width = 640
        self.height = 480
        self.near_val = 0.1
        self.far_val = 10.0
    
    def get_depth_image(self):
        """
        Capture depth image using PyBullet's depth camera functionality
        
        Returns:
            numpy.ndarray: Depth image with distance information
        """
        # Get camera link state
        link_state = p.getLinkState(self.robot_id, self.camera_link_index)
        camera_pos = link_state[0]
        camera_orient = link_state[1]
        
        # Compute view matrix
        view_matrix = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=camera_pos,
            distance=1.0,
            yaw=0,
            pitch=0,
            roll=0,
            upAxisIndex=2
        )
        
        # Compute projection matrix
        proj_matrix = p.computeProjectionMatrixFOV(
            fov=60,
            aspect=self.width/self.height,
            nearVal=self.near_val,
            farVal=self.far_val
        )
        
        # Capture depth image
        width, height, depth_buffer, _, _ = p.getCameraImage(
            width=self.width,
            height=self.height,
            viewMatrix=view_matrix,
            projectionMatrix=proj_matrix,
            renderer=p.ER_TINY_RENDERER  # Use tiny renderer to ensure depth capture
        )
        
        # Convert depth buffer to depth image
        depth_image = np.reshape(depth_buffer, (height, width))
        
        # Normalize depth image to actual depth values
        depth_image = self.convert_depth_buffer_to_depth(depth_image)
        
        return depth_image
    
    def convert_depth_buffer_to_depth(self, depth_buffer):
        """
        Convert depth buffer to actual depth values
        
        Args:
            depth_buffer (numpy.ndarray): Raw depth buffer from PyBullet
        
        Returns:
            numpy.ndarray: Normalized depth values
        """
        # Convert depth buffer to actual depth values
        depth_image = self.far_val * self.near_val / (self.far_val - (depth_buffer * (self.far_val - self.near_val)))
        return depth_image
    
    def detect_objects_from_depth(self, depth_image, threshold_distance=1.5):
        """
        Detect objects using depth image processing
        
        Args:
            depth_image (numpy.ndarray): Depth image
            threshold_distance (float): Max distance for object detection
        
        Returns:
            list: Detected object coordinates and details
        """
        # Simple object detection by finding clusters in depth image
        detected_objects = []
        
        # Threshold the depth image
        object_mask = depth_image < threshold_distance
        
        # Find connected components (potential objects)
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            object_mask.astype(np.uint8), connectivity=8
        )
        
        for i in range(1, num_labels):  # Skip background
            if stats[i, cv2.CC_STAT_AREA] > 100:  # Minimum object size
                # Convert image coordinates to 3D world coordinates
                u, v = centroids[i]
                z = depth_image[int(v), int(u)]
                
                # Compute 3D coordinates using pinhole camera model
                # Note: These calculations might need adjustment based on actual camera parameters
                fx = fy = 500.0  # Focal lengths
                cx, cy = self.width / 2, self.height / 2
                
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy
                
                detected_objects.append({
                    'position': [x, y, z],
                    'size': stats[i, cv2.CC_STAT_AREA],
                    'type': 'unknown'
                })
        
        return detected_objects

class ObjectClassificationNetwork(nn.Module):
    """
    Simple neural network for object classification
    """
    def __init__(self, input_size=3, num_classes=5):
        super().__init__()
        self.network = nn.Sequential(
            nn.Linear(input_size, 64),
            nn.ReLU(),
            nn.Linear(64, 32),
            nn.ReLU(),
            nn.Linear(32, num_classes)
        )
    
    def forward(self, x):
        return self.network(x)

class RobotInteractionSystem:
    def __init__(self, robot_id, controllable_joints):
        self.robot_id = robot_id
        self.controllable_joints = controllable_joints
        self.classifier = ObjectClassificationNetwork()
    
    def move_end_effector(self, target_position):
        """
        Move robot end effector to target position
        """
        joint_poses = p.calculateInverseKinematics(
            self.robot_id, 
            self.controllable_joints[-1], 
            target_position
        )
        
        for joint, pose in zip(self.controllable_joints, joint_poses):
            p.setJointMotorControl2(
                bodyUniqueId=self.robot_id,
                jointIndex=joint,
                controlMode=p.POSITION_CONTROL,
                targetPosition=pose
            )
    
    def classify_object(self, object_features):
        """
        Classify detected object using neural network
        """
        with torch.no_grad():
            features_tensor = torch.tensor(object_features, dtype=torch.float32)
            prediction = self.classifier(features_tensor)
            class_id = torch.argmax(prediction).item()
        
        object_classes = ['small', 'medium', 'large', 'tool', 'container']
        return object_classes[class_id]
    
    def pick_object(self, object_details):
        """
        Pick object based on classification and position
        """
        # Move to hover position
        hover_pos = list(object_details['position'])
        hover_pos[2] += 0.1  # Hover slightly above object
        
        self.move_end_effector(hover_pos)
        time.sleep(1)  # Wait for movement
        
        # Move to actual object position
        self.move_end_effector(object_details['position'])
        time.sleep(1)
        
        print(f"Picked {object_details['type']} object")

def setup_pybullet_environment():
    """
    Setup PyBullet simulation environment
    """
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    
    # Load plane and table
    planeId = p.loadURDF("plane.urdf")
    tableId = p.loadURDF("table/table.urdf", [0.5, 0, 0])
    
    # Load robot
    robot_urdf = "kinova-ros/kinova_description/urdf/j2s7s300_standalone.urdf"
    robotId = p.loadURDF(robot_urdf, [0, 0, 1], useFixedBase=True)
    
    # Get controllable joints
    controllable_joints = []
    for i in range(p.getNumJoints(robotId)):
        joint_info = p.getJointInfo(robotId, i)
        if joint_info[2] != p.JOINT_FIXED:
            controllable_joints.append(i)
    
    return robotId, controllable_joints

def main():
    # Setup environment
    robotId, controllable_joints = setup_pybullet_environment()
    
    # Create depth sensor (assuming last link is camera)
    depth_sensor = DepthSensorSimulator(robotId, p.getNumJoints(robotId) - 1)
    
    # Create robot interaction system
    robot_system = RobotInteractionSystem(robotId, controllable_joints)
    
    # Simulation loop
    try:
        for _ in range(1000):  # Simulate 1000 steps
            # Capture depth image
            depth_image = depth_sensor.get_depth_image()
            
            # Detect objects
            detected_objects = depth_sensor.detect_objects_from_depth(depth_image)
            
            for obj in detected_objects:
                # Classify object
                obj['type'] = robot_system.classify_object(obj['position'])
                
                # Pick objects
                if obj['type'] in ['tool', 'container']:
                    robot_system.pick_object(obj)
            
            p.stepSimulation()
            time.sleep(1/240)  # Maintain simulation speed
    
    except KeyboardInterrupt:
        print("Simulation stopped by user")
    finally:
        # Disconnect PyBullet
        p.disconnect()

if __name__ == "__main__":
    main()