import numpy as np
import pygame
import math
from threeR import ThreeRManipulator
from twoR import TwoRManipulator
import re

# # Set up the display
# WIDTH, HEIGHT = 800, 600
# screen = pygame.display.set_mode((WIDTH, HEIGHT))
# pygame.display.set_caption("2D Shape and Manipulator Trajectory Generator")

# # Define colors
# BLACK = (0, 0, 0)
# WHITE = (255, 255, 255)
# RED = (255, 0, 0)
# GREEN = (0, 255, 0)
# BLUE = (0, 0, 255)

def create_cube(size=1):
    points = np.array([
        [0, 0], [0, size], [size, 0], [size, size]
    ])
    return points

def create_circle(radius=1, resolution=20):
    theta = np.linspace(0, 2 * np.pi, resolution)
    x = radius * np.cos(theta)
    y = radius * np.sin(theta)
    points = np.column_stack((x, y))
    return points

def create_triangle(base_size=1, height=1):
    points = np.array([
        [0, 0], [base_size, 0], [base_size/2, height]
    ])
    return points

def generate_shape(prompt, manipulator):
    prompt = prompt.lower()
    
    if 'cube' in prompt or 'box' in prompt:
        size = extract_number(prompt)
        points = create_cube(size)
        if 'two' in prompt.lower() or '2r' in prompt.lower():
            manipulator = TwoRManipulator()
            print(points[0],points[1])
            q1,q2= manipulator.inverse_kinematics(points[0],points[1])
            manipulator.draw_manipulator(q1,q2)


        else:
            manipulator = ThreeRManipulator()
            q1,q2,q3 = manipulator.inverse_kinematics(points[0],points[1])
            manipulator.draw_manipulator(q1,q2,q3)
    
    elif 'circle' in prompt or 'ball' in prompt:
        radius = extract_number(prompt)
        points = create_circle(radius)
        if 'two' in prompt.lower() or '2r' in prompt.lower():
            manipulator = TwoRManipulator()
            q1,q2= manipulator.inverse_kinematics(points[0],points[1])
            manipulator.draw_manipulator(q1,q2)
        else:
            manipulator = ThreeRManipulator()
            q1,q2,q3 = manipulator.inverse_kinematics(points[0],points[1])
            manipulator.draw_manipulator(q1,q2,q3)
    
    elif 'triangle' in prompt:
        base_size = extract_number(prompt)
        height_match = re.search(r'height\s+(\d+(?:\.\d+)?)', prompt)
        height = float(height_match.group(1)) if height_match else base_size
        points = create_triangle(base_size, height)
        if 'two' in prompt.lower() or '2r' in prompt.lower():
            manipulator = TwoRManipulator()
            q1,q2= manipulator.inverse_kinematics(points[0],points[1])
            manipulator.draw_manipulator(q1,q2)
        else:
            manipulator = ThreeRManipulator()
            q1,q2,q3 = manipulator.inverse_kinematics(points[0],points[1])
            manipulator.draw_manipulator(q1,q2,q3)
    
    else:
        print("I don't understand that shape. Try 'cube', 'circle', or 'triangle' with size/radius specifications.")

    manipulator.run()

def extract_number(prompt, default=1):
    match = re.search(r'\b(\d+(?:\.\d+)?)\s*(?:unit|radius|size|height)?\b', prompt)
    return float(match.group(1)) if match else default

# Main loop
print("Welcome to the 2D Shape and Manipulator Trajectory Generator!")
print("You can create cubes, circles, and triangles with custom sizes, and trace the trajectory using a 2R or 3R manipulator.")
print("Examples:")
print("- 'Create a cube with size 2 using the 3R manipulator'")
print("- 'I want a circle of 3 unit radius traced by the 2R manipulator'")
print("- 'Generate a triangle with base 2 and height 3 using the 3R manipulator'")
print("Type 'quit' to exit.")

while True:
    user_input = input("\nDescribe a shape and the manipulator: ")
    if user_input.lower() == 'quit':
        break
    
    generate_shape(user_input, None)

print("Thank you for using the 2D Shape and Manipulator Trajectory Generator!")