import pygame
import math

class TwoRManipulator:
    def __init__(self, width=800, height=600):
        # Initialize Pygame
        pygame.init()

        # Set up the display
        self.width, self.height = width, height
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("2R Manipulator")

        # Define colors
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.red = (255, 0, 0)
        self.green = (0, 255, 0)

        # Define the manipulator parameters
        self.base_x, self.base_y = self.width // 2, self.height // 2
        self.link1_length = 150
        self.link2_length = 100
        self.joint_radius = 10

        # Initial joint angles (in radians)
        self.joint_angle_1 = 0
        self.joint_angle_2 = 0

        # Target position
        self.target_x, self.target_y = None, None

    def inverse_kinematics(self, x, y):
        x = x - self.base_x
        y = y - self.base_y

        # Calculate the distance from the base to the target position
        distance = math.sqrt((x) ** 2 + (y) ** 2)

        # Check if the target position is within the workspace
        if distance > self.link1_length + self.link2_length or distance < abs(self.link1_length - self.link2_length):
            return (0, 0)

        # Calculate the joint angles
        try:
            cos_q2 = (x ** 2 + y ** 2 - (self.link1_length ** 2 + self.link2_length ** 2)) / (2 * self.link1_length * self.link2_length)
            if cos_q2 < -1 or cos_q2 > 1:
                return (0, 0)  # Target position is outside the valid range
            q2 = math.acos(cos_q2)
            q1 = math.atan2(y, x) - math.atan2(self.link2_length * math.sin(q2), self.link1_length + self.link2_length * math.cos(q2))
            return q1, q2
        except ValueError:
            # Handle math domain error
            return (0, 0)

    def draw_manipulator(self, q1, q2):
        # Clear the screen
        self.screen.fill(self.white)

        # Draw the first link
        link1_end_x = self.base_x + self.link1_length * math.cos(q1)
        link1_end_y = self.base_y + self.link1_length * math.sin(q1)
        pygame.draw.line(self.screen, self.black, (self.base_x, self.base_y), (link1_end_x, link1_end_y), 5)
        pygame.draw.circle(self.screen, self.red, (link1_end_x, link1_end_y), self.joint_radius)

        # Draw the second link
        link2_end_x = link1_end_x + self.link2_length * math.cos(q1 + q2)
        link2_end_y = link1_end_y + self.link2_length * math.sin(q1 + q2)
        pygame.draw.line(self.screen, self.black, (link1_end_x, link1_end_y), (link2_end_x, link2_end_y), 5)
        pygame.draw.circle(self.screen, self.green, (link2_end_x, link2_end_y), self.joint_radius // 2)

        # Draw the target position
        if self.target_x is not None and self.target_y is not None:
            pygame.draw.circle(self.screen, self.red, (self.target_x, self.target_y), 5)

        # Update the display
        pygame.display.flip()

    def run(self):
        running = True
        while running:
            # Handle events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    self.target_x, self.target_y = event.pos
                    self.joint_angle_1, self.joint_angle_2 = self.inverse_kinematics(self.target_x, self.target_y)
                    self.draw_manipulator(self.joint_angle_1, self.joint_angle_2)
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE:
                        self.joint_angle_1, self.joint_angle_2 = 0, 0  # Reset joint angles to default position
                        self.draw_manipulator(self.joint_angle_1, self.joint_angle_2)

        # Quit Pygame
        pygame.quit()

# Example usage
if __name__ == "__main__":
    manipulator = TwoRManipulator()
    manipulator.run()