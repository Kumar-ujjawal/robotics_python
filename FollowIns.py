import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import ConvexHull
import re
from threeR import ThreeRManipulator
from twoR import TwoRManipulator

def create_cube(size=1):
    points = np.array([
        [0, 0, 0], [0, 0, size], [0, size, 0], [0, size, size],
        [size, 0, 0], [size, 0, size], [size, size, 0], [size, size, size]
    ])
    return points

def create_sphere(radius=1, resolution=20):
    phi = np.linspace(0, np.pi, resolution)
    theta = np.linspace(0, 2 * np.pi, resolution)
    phi, theta = np.meshgrid(phi, theta)
    
    x = radius * np.sin(phi) * np.cos(theta)
    y = radius * np.sin(phi) * np.sin(theta)
    z = radius * np.cos(phi)
    
    points = np.vstack((x.flatten(), y.flatten(), z.flatten())).T
    return points

def create_pyramid(base_size=1, height=1):
    points = np.array([
        [0, 0, 0], [base_size, 0, 0], [base_size, base_size, 0], [0, base_size, 0],
        [base_size/2, base_size/2, height]
    ])
    return points

def extract_number(prompt, default=1):
    match = re.search(r'\b(\d+(?:\.\d+)?)\s*(?:unit|radius|size|height)?\b', prompt)
    return float(match.group(1)) if match else default

def generate_shape(prompt, manipulator):
    prompt = prompt.lower()
    
    if 'cube' in prompt or 'box' in prompt:
        size = extract_number(prompt)
        points = create_cube(size)
        trajectory_points = [tuple(p) for p in points]
        manipulator.trajectory_points = trajectory_points
        manipulator.draw_manipulator(manipulator.joint_angles)
    
    elif 'sphere' in prompt or 'ball' in prompt:
        radius = extract_number(prompt)
        points = create_sphere(radius)
        trajectory_points = [tuple(p) for p in points]
        manipulator.trajectory_points = trajectory_points
        manipulator.draw_manipulator(manipulator.joint_angles)
    
    elif 'pyramid' in prompt:
        base_size = extract_number(prompt)
        height_match = re.search(r'height\s+(\d+(?:\.\d+)?)', prompt)
        height = float(height_match.group(1)) if height_match else base_size
        points = create_pyramid(base_size, height)
        trajectory_points = [tuple(p) for p in points]
        manipulator.trajectory_points = trajectory_points
        manipulator.draw_manipulator(manipulator.joint_angles)
    
    else:
        print("I don't understand that shape. Try 'cube', 'sphere', or 'pyramid' with size/radius specifications.")

# Main loop
print("Welcome to the 3D Shape Generator and Manipulator Controller!")
print("You can create cubes, spheres, and pyramids with custom sizes, and trace the trajectory using a 2R or 3R manipulator.")
print("Examples:")
print("- 'Create a cube with size 2 using the 3R manipulator'")
print("- 'I want a sphere of 3 unit radius traced by the 2R manipulator'")
print("- 'Generate a pyramid with base 2 and height 3 using the 3R manipulator'")
print("Type 'quit' to exit.")

while True:
    user_input = input("\nDescribe a shape and the manipulator: ")
    if user_input.lower() == 'quit':
        break
    
    if 'two' in user_input.lower():
        manipulator = TwoRManipulator()
    else:
        manipulator = ThreeRManipulator()
    
    generate_shape(user_input, manipulator)
    manipulator.run()

print("Thank you for using the 3D Shape Generator and Manipulator Controller!")