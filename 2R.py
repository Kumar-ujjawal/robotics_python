import pygame
import math

# Initialize Pygame
pygame.init()

# Set up the display
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("2R Manipulator")

# Define colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)

# Define the manipulator parameters
BASE_X, BASE_Y = WIDTH // 2, HEIGHT // 2
LINK1_LENGTH = 150
LINK2_LENGTH = 100
JOINT_RADIUS = 10

# Initial joint angles (in radians)
JOINT_ANGLE_1 = 0
JOINT_ANGLE_2 = 0

# Target position
TARGET_X, TARGET_Y = None, None

# Function to calculate the inverse kinematics
def inverse_kinematics(x, y):
    x = x- BASE_X
    y = y - BASE_Y
    

    # Calculate the distance from the base to the target position
    distance = math.sqrt((x) ** 2 + (y) ** 2)

    # Check if the target position is within the workspace
    if distance > LINK1_LENGTH + LINK2_LENGTH or distance < abs(LINK1_LENGTH - LINK2_LENGTH):
        return (0,0)

    # Calculate the joint angles
    try:
        cos_q2 = (x ** 2 + y ** 2 - (LINK1_LENGTH ** 2 + LINK2_LENGTH ** 2)) / (2 * LINK1_LENGTH * LINK2_LENGTH)
        print(cos_q2)
        if cos_q2 < -1 or cos_q2 > 1:
            return (0,0)  # Target position is outside the valid range
        q2 = math.acos(cos_q2)
        q1 = math.atan2(y, x) - math.atan2(LINK2_LENGTH * math.sin(q2), LINK1_LENGTH + LINK2_LENGTH * math.cos(q2))
        print(q1,q2)
        
        return q1, q2
    except ValueError:
        # Handle math domain error
        return (0,0)

# Function to draw the manipulator
def draw_manipulator(q1, q2):
    # Clear the screen
    screen.fill(WHITE)
    # print(q1,q2)
    # Draw the first link
    link1_end_x = BASE_X + LINK1_LENGTH * math.cos(q1)
    link1_end_y = BASE_Y + LINK1_LENGTH * math.sin(q1)
    pygame.draw.line(screen, BLACK, (BASE_X, BASE_Y), (link1_end_x, link1_end_y), 5)
    pygame.draw.circle(screen, RED, (link1_end_x, link1_end_y), JOINT_RADIUS)

    # Draw the second link
    link2_end_x = link1_end_x + LINK2_LENGTH * math.cos(q1 + q2)
    link2_end_y = link1_end_y + LINK2_LENGTH * math.sin(q1 + q2)
    pygame.draw.line(screen, BLACK, (link1_end_x, link1_end_y), (link2_end_x, link2_end_y), 5)
    pygame.draw.circle(screen, GREEN, (link2_end_x, link2_end_y), JOINT_RADIUS // 2)

    # Draw the target position
    if TARGET_X is not None and TARGET_Y is not None:
        pygame.draw.circle(screen, RED, (TARGET_X, TARGET_Y), 5)

    # Update the display
    pygame.display.flip()

# Game loop
running = True
while running:
    # Handle events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            TARGET_X, TARGET_Y = event.pos
            # print(TARGET_X,TARGET_Y)
            JOINT_ANGLE_1, JOINT_ANGLE_2 = inverse_kinematics(TARGET_X, TARGET_Y)
            # print(JOINT_ANGLE_1,JOINT_ANGLE_2)
            draw_manipulator(JOINT_ANGLE_1, JOINT_ANGLE_2)
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                JOINT_ANGLE_1, JOINT_ANGLE_2 = 0,0  # Reset joint angles to default position
                draw_manipulator(JOINT_ANGLE_1, JOINT_ANGLE_2)

    # Draw the manipulator
#     draw_manipulator(JOINT_ANGLE_1, JOINT_ANGLE_2)

# Quit Pygame
pygame.quit()