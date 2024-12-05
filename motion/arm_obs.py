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
        self.GRAY = (128, 128, 128)
        
        # Manipulator parameters
        self.BASE_X = self.WIDTH // 2
        self.BASE_Y = self.HEIGHT // 2
        self.LINK1_LENGTH = 150
        self.LINK2_LENGTH = 100
        self.JOINT_RADIUS = 10
        
        # Workspace parameters
        self.MIN_REACH = abs(self.LINK1_LENGTH - self.LINK2_LENGTH)
        self.MAX_REACH = self.LINK1_LENGTH + self.LINK2_LENGTH
        
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
        
        # Initialize obstacles (circular for simplicity)
        self.obstacles = [
            {"x": 200, "y": 200, "radius": 50},
            {"x": 600, "y": 400, "radius": 40},
            {"x": 400, "y": 500, "radius": 45}
        ]
        
    def is_point_in_workspace(self, x: int, y: int) -> bool:
        # Calculate distance from base
        dx = x - self.BASE_X
        dy = y - self.BASE_Y
        distance = math.sqrt(dx ** 2 + dy ** 2)
        
        # Check if point is within the annular workspace
        return self.MIN_REACH <= distance <= self.MAX_REACH
        
    def check_collision(self, point: Tuple[int, int]) -> bool:
        # Check collision with obstacles
        x, y = point
        for obstacle in self.obstacles:
            dist = math.sqrt((x - obstacle["x"])**2 + (y - obstacle["y"])**2)
            if dist <= obstacle["radius"]:
                return True
        return False
        
    def check_line_collision(self, start: Tuple[int, int], end: Tuple[int, int]) -> bool:
        # Check if line segment intersects with any obstacle
        steps = int(math.sqrt((end[0]-start[0])**2 + (end[1]-start[1])**2) / 5)
        steps = max(steps, 1)
        
        for i in range(steps + 1):
            t = i / steps
            x = start[0] + t * (end[0] - start[0])
            y = start[1] + t * (end[1] - start[1])
            if self.check_collision((int(x), int(y))):
                return True
        return False
        
    def inverse_kinematics(self, x: int, y: int) -> Tuple[float, float]:
        x = x - self.BASE_X
        y = y - self.BASE_Y
        distance = math.sqrt(x ** 2 + y ** 2)
        
        if distance > self.MAX_REACH or distance < self.MIN_REACH:
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
        
    def draw_workspace(self):
        # Draw workspace boundaries
        pygame.draw.circle(self.screen, self.GRAY, (self.BASE_X, self.BASE_Y), 
                         self.MAX_REACH, 1)
        pygame.draw.circle(self.screen, self.GRAY, (self.BASE_X, self.BASE_Y), 
                         self.MIN_REACH, 1)
        
    def draw_obstacles(self):
        # Draw obstacles
        for obstacle in self.obstacles:
            pygame.draw.circle(self.screen, self.RED, 
                             (obstacle["x"], obstacle["y"]), 
                             obstacle["radius"])
        
    def draw_manipulator(self, q1: float, q2: float):
        self.screen.fill(self.WHITE)
        
        # Draw workspace
        self.draw_workspace()
        
        # Draw obstacles
        self.draw_obstacles()
        
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
                angle = random.uniform(0, 2 * math.pi)
                radius = random.uniform(self.MIN_REACH, self.MAX_REACH)
                sample = (
                    int(self.BASE_X + radius * math.cos(angle)),
                    int(self.BASE_Y + radius * math.sin(angle))
                )
            
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
            
            # Check if new node is valid
            if (self.inverse_kinematics(new_x, new_y) is not None and
                not self.check_collision(new_node) and
                not self.check_line_collision(nearest, new_node)):
                
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
        print("Click on the screen to set the goal position...")
        font = pygame.font.Font(None, 36)
        
        running = True
        goal_set = False
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                    
                elif event.type == pygame.MOUSEBUTTONDOWN and not goal_set:
                    goal_pos = event.pos
                    
                    # Check if goal is in workspace and not in collision
                    if not self.is_point_in_workspace(goal_pos[0], goal_pos[1]):
                        message = "Goal is outside workspace! Click again."
                        text = font.render(message, True, self.RED)
                        self.screen.blit(text, (10, 10))
                        pygame.display.flip()
                        continue
                        
                    if self.check_collision(goal_pos):
                        message = "Goal is in collision! Click again."
                        text = font.render(message, True, self.RED)
                        self.screen.blit(text, (10, 10))
                        pygame.display.flip()
                        continue
                    
                    start_pos = self.forward_kinematics(self.joint_angle_1, self.joint_angle_2)
                    
                    # Plan path using RRT
                    print("Planning path...")
                    self.path = self.plan_rrt_path(start_pos, goal_pos)
                    
                    if self.path is None:
                        message = "No path found! Click again."
                        text = font.render(message, True, self.RED)
                        self.screen.blit(text, (10, 10))
                        pygame.display.flip()
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