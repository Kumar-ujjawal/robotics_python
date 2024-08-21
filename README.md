## project description

# 3D Shape and Manipulator Trajectory Generator

This repository contains code for generating 3D shapes and controlling the trajectory of 2R and 3R manipulators based on simple text-based commands.

## Features

- **3D Shape Generation**: Generate the following 3D shapes:
  - Cube
  - Sphere
  - Pyramid

- **Manipulator Trajectory Control**: Use the generated 3D shapes as trajectory points for either a 2R or 3R manipulator. Users can choose the manipulator through text commands.

- **Pygame Visualization**: Visualize the 3D shapes and the manipulator movements in real-time using the Pygame library.

## Getting Started

### Clone the Repository

```
git clone https://github.com/Kumar-ujjawal/robotics_python.git

```

## Install the Required Dependencies
Make sure you have Python 3.x installed. Then install the required Python libraries:


```
pip install numpy matplotlib pygame scipy
```

Run the Main Script
```
python main.py
```
Usage
After running main.py, you will see the following instructions:

text
Welcome to the 3D Shape Generator and Manipulator Controller!
You can create cubes, spheres, and pyramids with custom sizes, and trace the trajectory using a 2R or 3R manipulator.
Examples:
- 'Create a cube with size 2 using the 3R manipulator'
- 'I want a sphere of 3 unit radius traced by the 2R manipulator'
- 'Generate a pyramid with base 2 and height 3 using the 3R manipulator'
Type 'quit' to exit.
You can then enter a command to describe the shape you want to generate and the manipulator you want to use. For example:

text

```
Describe a shape and the manipulator: Create a cube with size 3 using the 3R manipulator
The program will generate the cube shape and display the 3R manipulator tracing the trajectory of the cube.
```

Future Plans
Natural Language Interaction: Integrate large language models (LLMs) for more natural and flexible interaction, allowing users to describe their desired shapes and manipulator tasks using natural language.

Expanded Shape Generation: Expand the shape generation capabilities to include more complex 3D models and potentially integrate with CAD software or other 3D modeling tools.

Contributing
Contributions are welcome! If you encounter any issues or have suggestions for improvements, feel free to open an issue or submit a pull request.

Note: This project is in active development, and additional features and improvements will be implemented over time. Stay tuned for updates!