# draft one to text to 3D-design #
# The code works by taking inputs as text commands and generate the pre defiend shapes, to be updated by DL models later #
# -----------------------------------------------------------------------------------------------------------------------# 

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import ConvexHull
import re

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

def plot_3d_shape(points, title):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    hull = ConvexHull(points)
    for simplex in hull.simplices:
        ax.plot3D(points[simplex, 0], points[simplex, 1], points[simplex, 2], 'r-')
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(title)
    plt.show()

def extract_number(prompt, default=1):
    match = re.search(r'\b(\d+(?:\.\d+)?)\s*(?:unit|radius|size|height)?\b', prompt)
    return float(match.group(1)) if match else default

def generate_shape(prompt):
    prompt = prompt.lower()
    
    if 'cube' in prompt or 'box' in prompt:
        size = extract_number(prompt)
        points = create_cube(size)
        plot_3d_shape(points, f"Cube (size: {size})")
    
    elif 'sphere' in prompt or 'ball' in prompt:
        radius = extract_number(prompt)
        points = create_sphere(radius)
        plot_3d_shape(points, f"Sphere (radius: {radius})")
    
    elif 'pyramid' in prompt:
        base_size = extract_number(prompt)
        height_match = re.search(r'height\s+(\d+(?:\.\d+)?)', prompt)
        height = float(height_match.group(1)) if height_match else base_size
        points = create_pyramid(base_size, height)
        plot_3d_shape(points, f"Pyramid (base: {base_size}, height: {height})")
    
    else:
        print("I don't understand that shape. Try 'cube', 'sphere', or 'pyramid' with size/radius specifications.")

# Main loop
print("Welcome to the 3D Shape Generator!")
print("You can create cubes, spheres, and pyramids with custom sizes.")
print("Examples:")
print("- 'Create a cube with size 2'")
print("- 'I want a sphere of 3 unit radius'")
print("- 'Generate a pyramid with base 2 and height 3'")
print("Type 'quit' to exit.")

while True:
    user_input = input("\nDescribe a shape: ")
    if user_input.lower() == 'quit':
        break
    generate_shape(user_input)

print("Thank you for using the 3D Shape Generator!")