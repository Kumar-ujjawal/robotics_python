import pygame
import math
import numpy as np
from scipy.optimize import least_squares

class ThreeRManipulator:
    def __init__(self, width=800, height=600):
        pygame.init()

        # Set up the display
        self.width, self.height = width, height
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("3R Manipulator")

        # Define colors
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.red = (255, 0, 0)
        self.green = (0, 255, 0)
        self.blue = (0, 0, 255)

        # Define the manipulator parameters
        self.base_x, self.base_y = self.width // 2, self.height // 2
        self.link1_length = 120
        self.link2_length = 100
        self.link3_length = 80
        self.joint_radius = 10
        self.joint_angles = np.array([0, 0, 0])

        # Target position
        self.target_x, self.target_y = None, None
        self.trajectory_points = []

    def forward_kinematics(self, joint_angles):
        q1, q2, q3 = joint_angles
        x = self.link1_length * math.cos(q1) + self.link2_length * math.cos(q1 + q2) + self.link3_length * math.cos(q1 + q2 + q3)
        y = self.link1_length * math.sin(q1) + self.link2_length * math.sin(q1 + q2) + self.link3_length * math.sin(q1 + q2 + q3)
        return np.array([x, y])

    def objective_function(self, joint_angles, target_position):
        current_position = self.forward_kinematics(joint_angles)
        return np.linalg.norm(current_position - target_position)

    def inverse_kinematics(self, target_position, initial_guess):
        result = least_squares(
            self.objective_function,
            initial_guess,
            args=(target_position,),
            bounds=(-np.pi, np.pi)
        )
        return result.x

    def draw_manipulator(self, joint_angles):
        q1, q2, q3 = joint_angles
        
        # Clear the screen
        self.screen.fill(self.white)

        # Draw the first link
        link1_end_x = self.base_x + self.link1_length * math.cos(q1)
        link1_end_y = self.base_y + self.link1_length * math.sin(q1)
        pygame.draw.line(self.screen, self.black, (self.base_x, self.base_y), (link1_end_x, link1_end_y), 5)
        pygame.draw.circle(self.screen, self.red, (int(link1_end_x), int(link1_end_y)), self.joint_radius)

        # Draw the second link
        link2_end_x = link1_end_x + self.link2_length * math.cos(q1 + q2)
        link2_end_y = link1_end_y + self.link2_length * math.sin(q1 + q2)
        pygame.draw.line(self.screen, self.black, (link1_end_x, link1_end_y), (link2_end_x, link2_end_y), 5)
        pygame.draw.circle(self.screen, self.green, (int(link2_end_x), int(link2_end_y)), self.joint_radius)

        # Draw the third link
        link3_end_x = link2_end_x + self.link3_length * math.cos(q1 + q2 + q3)
        link3_end_y = link2_end_y + self.link3_length * math.sin(q1 + q2 + q3)
        pygame.draw.line(self.screen, self.black, (link2_end_x, link2_end_y), (link3_end_x, link3_end_y), 5)
        pygame.draw.circle(self.screen, self.blue, (int(link3_end_x), int(link3_end_y)), self.joint_radius // 2)

        # Draw the target position
        if self.target_x is not None and self.target_y is not None:
            pygame.draw.circle(self.screen, self.red, (self.target_x, self.target_y), 5)

        # Draw the trajectory
        for point in self.trajectory_points:
            pygame.draw.circle(self.screen, self.red, point, 3)

        # Update the display
        pygame.display.flip()

    def run(self):
        running = True
        while running:
            # Handle events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.MOUSEMOTION:
                    self.target_x, self.target_y = event.pos
                    target_position = np.array([self.target_x - self.base_x, self.target_y - self.base_y])
                    self.joint_angles = self.inverse_kinematics(target_position, self.joint_angles)
                    self.trajectory_points = self.interpolate_trajectory((self.base_x, self.base_y), (self.target_x, self.target_y), 10)
                    self.draw_manipulator(self.joint_angles)
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE:
                        self.joint_angles = np.array([0, 0, 0])  # Reset joint angles to default position
                        self.trajectory_points = []
                        self.draw_manipulator(self.joint_angles)

    def interpolate_trajectory(self, start, end, steps):
        x1, y1 = start
        x2, y2 = end
        dx = x2 - x1
        dy = y2 - y1
        distance = math.sqrt(dx ** 2 + dy ** 2)
        if distance == 0:
            return []
        dx /= distance
        dy /= distance
        trajectory = []
        for i in range(steps):
            x = int(x1 + dx * i * distance / (steps - 1))
            y = int(y1 + dy * i * distance / (steps - 1))
            trajectory.append((x, y))
        return trajectory


if __name__ == "__main__":
    manipulator = ThreeRManipulator()
    manipulator.run()