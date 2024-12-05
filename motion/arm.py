import pygame
import math
import random
from typing import Tuple, List

class ManipulatorRRTPlanner:
    def __init__(self):
        # Initialize Pygame
        pygame.init()
        
        # Display settings
        self.WIDTH = 800
        self.HEIGHT = 600
        self.screen = pygame.display.set_mode((self.WIDTH, self.HEIGHT))
        pygame.display.set_caption("2R Manipulator with RRT Path Planning")
        
        # Colors
        self.BLACK = (0, 0, 0)
        self.WHITE = (255, 255, 255)
        self.RED = (255, 0, 0)
        self.GREEN = (0, 255, 0)
        self.BLUE = (0, 0, 255)
        
        # Manipulator parameters
        self.BASE_X = self.WIDTH // 2
        self.BASE_Y = self.HEIGHT // 2
        self.LINK1_LENGTH = 150
        self.LINK2_LENGTH = 100
        self.JOINT_RADIUS = 10
        
        # RRT parameters
        self.MAX_ITERATIONS = 5000
        self.STEP_SIZE = 20
        self.GOAL_SAMPLE_RATE = 0.1
        
        # Initialize joint angles
        self.joint_angle_1 = 0
        self.joint_angle_2 = 0
        
        # Path planning variables
        self.path = []
        self.current_path_index = 0
        
    def inverse_kinematics(self, x: int, y: int) -> Tuple[float, float]:
        x = x - self.BASE_X
        y = y - self.BASE_Y
        distance = math.sqrt(x ** 2 + y ** 2)
        
        if distance > self.LINK1_LENGTH + self.LINK2_LENGTH or \
           distance < abs(self.LINK1_LENGTH - self.LINK2_LENGTH):
            return None
        
        try:
            cos_q2 = (x ** 2 + y ** 2 - (self.LINK1_LENGTH ** 2 + self.LINK2_LENGTH ** 2)) / \
                     (2 * self.LINK1_LENGTH * self.LINK2_LENGTH)
            if cos_q2 < -1 or cos_q2 > 1:
                return None
                
            q2 = math.acos(cos_q2)
            q1 = math.atan2(y, x) - math.atan2(self.LINK2_LENGTH * math.sin(q2),
                                              self.LINK1_LENGTH + self.LINK2_LENGTH * math.cos(q2))
            return q1, q2
        except:
            return None
            
    def forward_kinematics(self, q1: float, q2: float) -> Tuple[float, float]:
        x = self.LINK1_LENGTH * math.cos(q1) + self.LINK2_LENGTH * math.cos(q1 + q2)
        y = self.LINK1_LENGTH * math.sin(q1) + self.LINK2_LENGTH * math.sin(q1 + q2)
        return (x + self.BASE_X, y + self.BASE_Y)
        
    def draw_manipulator(self, q1: float, q2: float):
        self.screen.fill(self.WHITE)
        
        # Draw first link
        link1_end_x = self.BASE_X + self.LINK1_LENGTH * math.cos(q1)
        link1_end_y = self.BASE_Y + self.LINK1_LENGTH * math.sin(q1)
        pygame.draw.line(self.screen, self.BLACK, (self.BASE_X, self.BASE_Y), 
                        (link1_end_x, link1_end_y), 5)
        pygame.draw.circle(self.screen, self.RED, (int(link1_end_x), int(link1_end_y)), 
                         self.JOINT_RADIUS)
        
        # Draw second link
        link2_end_x = link1_end_x + self.LINK2_LENGTH * math.cos(q1 + q2)
        link2_end_y = link1_end_y + self.LINK2_LENGTH * math.sin(q1 + q2)
        pygame.draw.line(self.screen, self.BLACK, (link1_end_x, link1_end_y), 
                        (link2_end_x, link2_end_y), 5)
        pygame.draw.circle(self.screen, self.GREEN, (int(link2_end_x), int(link2_end_y)), 
                         self.JOINT_RADIUS // 2)
        
        # Draw path
        if len(self.path) > 1:
            for i in range(len(self.path) - 1):
                pygame.draw.line(self.screen, self.BLUE, self.path[i], self.path[i + 1], 2)
        
        pygame.display.flip()
        
    def plan_rrt_path(self, start_pos: Tuple[int, int], goal_pos: Tuple[int, int]) -> List[Tuple[int, int]]:
        nodes = [start_pos]
        parents = [0]
        
        for _ in range(self.MAX_ITERATIONS):
            # Sample random point or goal
            if random.random() < self.GOAL_SAMPLE_RATE:
                sample = goal_pos
            else:
                sample = (random.randint(0, self.WIDTH), random.randint(0, self.HEIGHT))
            
            # Find nearest node
            min_dist = float('inf')
            nearest_idx = 0
            for i, node in enumerate(nodes):
                dist = math.sqrt((sample[0] - node[0])**2 + (sample[1] - node[1])**2)
                if dist < min_dist:
                    min_dist = dist
                    nearest_idx = i
            
            # Create new node
            nearest = nodes[nearest_idx]
            theta = math.atan2(sample[1] - nearest[1], sample[0] - nearest[0])
            new_x = nearest[0] + self.STEP_SIZE * math.cos(theta)
            new_y = nearest[1] + self.STEP_SIZE * math.sin(theta)
            new_node = (int(new_x), int(new_y))
            
            # Check if new node is valid (within workspace and IK solution exists)
            if self.inverse_kinematics(new_x, new_y) is not None:
                nodes.append(new_node)
                parents.append(nearest_idx)
                
                # Check if goal is reached
                if math.sqrt((new_x - goal_pos[0])**2 + (new_y - goal_pos[1])**2) < self.STEP_SIZE:
                    path = [goal_pos]
                    current_idx = len(nodes) - 1
                    while current_idx != 0:
                        path.append(nodes[current_idx])
                        current_idx = parents[current_idx]
                    path.append(start_pos)
                    return path[::-1]
        
        return None
        
    def run(self):
        # Get goal position from user
        print("Click on the screen to set the goal position...")
        
        running = True
        goal_set = False
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                    
                elif event.type == pygame.MOUSEBUTTONDOWN and not goal_set:
                    goal_pos = event.pos
                    start_pos = self.forward_kinematics(self.joint_angle_1, self.joint_angle_2)
                    
                    # Plan path using RRT
                    print("Planning path...")
                    self.path = self.plan_rrt_path(start_pos, goal_pos)
                    
                    if self.path is None:
                        print("No path found!")
                        continue
                        
                    print("Path found! Executing trajectory...")
                    goal_set = True
                    self.current_path_index = 0
            
            if goal_set and self.current_path_index < len(self.path):
                # Move manipulator along the path
                target = self.path[self.current_path_index]
                angles = self.inverse_kinematics(target[0], target[1])
                
                if angles is not None:
                    self.joint_angle_1, self.joint_angle_2 = angles
                    self.draw_manipulator(self.joint_angle_1, self.joint_angle_2)
                    self.current_path_index += 1
                    pygame.time.wait(50)  # Add delay for visualization
            
            elif goal_set:
                # Path completed
                goal_set = False
                print("Goal reached! Click to set a new goal...")
            
            else:
                # Draw current state
                self.draw_manipulator(self.joint_angle_1, self.joint_angle_2)
        
        pygame.quit()

if __name__ == "__main__":
    planner = ManipulatorRRTPlanner()
    planner.run()