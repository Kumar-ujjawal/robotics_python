import pybullet as p
import pybullet_data
import numpy as np
import itertools
import csv

class BoundingVolume:
    def __init__(self, center, radius):
        self.center = np.array(center)  # Convert to numpy array
        self.radius = radius

class BVHNode:
    def __init__(self, bv, link_name, joint_index, left=None, right=None):
        self.bv = bv
        self.link_name = link_name
        self.joint_index = joint_index
        self.left = left
        self.right = right

def construct_bvh_from_urdf(urdf_file):
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    robot_id = p.loadURDF(urdf_file, useFixedBase=True)

    num_joints = p.getNumJoints(robot_id)
    link_name_to_index = {p.getJointInfo(robot_id, i)[12].decode('UTF-8'): i for i in range(num_joints)}
    
    bvh_nodes = {}
    root = None
    
    for link_name, link_index in link_name_to_index.items():
        link_state = p.getLinkState(robot_id, link_index)
        link_pose = link_state[0]
        link_extent = p.getAABB(robot_id, link_index)
        link_center = 0.5 * (np.array(link_extent[0]) + np.array(link_extent[1]))
        link_radius = np.linalg.norm(np.array(link_extent[1]) - np.array(link_extent[0])) / 2.0
        bv = BoundingVolume(center=link_center, radius=link_radius)
        
        bvh_node = BVHNode(bv, link_name, link_index)
        bvh_nodes[link_name] = bvh_node
        
        if not root:
            root = bvh_node
    
    for link_name, link_index in link_name_to_index.items():
        parent_index = p.getJointInfo(robot_id, link_index)[16]
        if parent_index != -1:
            parent_name = p.getJointInfo(robot_id, parent_index)[12].decode('UTF-8')
            if parent_name in bvh_nodes:
                if bvh_nodes[parent_name].left is None:
                    bvh_nodes[parent_name].left = bvh_nodes[link_name]
                else:
                    bvh_nodes[parent_name].right = bvh_nodes[link_name]
    
    return robot_id, root, bvh_nodes

def check_self_collision(robot_id, bvh_nodes):
    collisions = []
    for node1, node2 in itertools.combinations(bvh_nodes.values(), 2):
        if node1.joint_index != node2.joint_index:
            if intersect(node1.bv, node2.bv):
                collisions.append((node1.link_name, node2.link_name))
    return collisions

def intersect(bv1, bv2):
    dist = np.linalg.norm(bv1.center - bv2.center)
    return dist < (bv1.radius + bv2.radius)

def sample_joint_angles(robot_id, num_samples=1000):
    num_joints = p.getNumJoints(robot_id)
    joint_ranges = []
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot_id, i)
        if joint_info[3] > -1:  # Check if joint is not fixed
            joint_ranges.append(joint_info[8:10])
    
    for _ in range(num_samples):
        joint_angles = [np.random.uniform(low, high) for low, high in joint_ranges]
        yield joint_angles

def find_self_collisions(urdf_file, num_samples=1000):
    robot_id, root, bvh_nodes = construct_bvh_from_urdf(urdf_file)
    
    all_collisions = {}
    
    for joint_angles in sample_joint_angles(robot_id, num_samples):
        movable_joints = [j for j in range(p.getNumJoints(robot_id)) if p.getJointInfo(robot_id, j)[3] > -1]
        for i, angle in zip(movable_joints, joint_angles):
            p.resetJointState(robot_id, i, angle)
        
        p.stepSimulation()
        
        # Update BVH nodes
        for node in bvh_nodes.values():
            link_state = p.getLinkState(robot_id, node.joint_index)
            node.bv.center = np.array(link_state[0])
        
        collisions = check_self_collision(robot_id, bvh_nodes)
        
        if collisions:
            collision_key = tuple(sorted(collisions))
            if collision_key not in all_collisions:
                all_collisions[collision_key] = joint_angles
    
    p.disconnect()
    return all_collisions

def save_collisions_to_csv(collisions, filename):
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Colliding Links', 'Joint Angles'])
        for collision, joint_angles in collisions.items():
            writer.writerow([str(collision), str(joint_angles)])

if __name__ == "__main__":
    urdf_file = r"kinova-ros/kinova_description/urdf/j2s7s300_standalone.urdf"
    output_file = "self_collisions.csv"
    
    try:
        collisions = find_self_collisions(urdf_file)
        
        if collisions:
            print("Self-collisions detected:")
            for collision, joint_angles in collisions.items():
                print(f"Colliding links: {collision}")
                print(f"Joint angles: {joint_angles}")
                print()
            
            save_collisions_to_csv(collisions, output_file)
            print(f"Collision data saved to {output_file}")
        else:
            print("No self-collisions detected.")
    except Exception as e:
        print(f"An error occurred: {e}")