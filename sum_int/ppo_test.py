import pybullet as p
import pybullet_data
import numpy as np
import torch
import time

from ppo import Agent, ActorNetwork, CriticNetwork  # Import your PPO agent and networks

# Constants
TIME_DELTA = 0.01  # Simulation time step
MAX_STEPS = 1000  # Maximum steps for testing
DISTANCE_THRESHOLD = 0.5  # Distance to goal considered successful

# Test function to evaluate model
def test_model(env, agent, target_position, max_steps=MAX_STEPS, distance_threshold=DISTANCE_THRESHOLD):
    observation = env.reset()
    success = False
    
    for step in range(max_steps):
        action, _ = agent.select_action(observation)
        observation, reward, done, _ = env.step(action)
        
        current_position = observation[-3:]  # Assuming last 3 elements are x, y, z
        distance = np.linalg.norm(current_position - target_position)
        
        if distance < distance_threshold:
            success = True
            return success, step + 1
        
        if done:
            break
    
    return success, step + 1

# Environment setup function for PyBullet
class PyBulletEnv:
    def __init__(self):
        self.physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        self.plane = p.loadURDF("plane.urdf")
        self.robot = p.loadURDF(r"kinova-ros\kinova_description\urdf\j2s7s300_standalone.urdf", basePosition=[0, 0, 0.1])
        self.observation_space = self.get_observation_space()
        self.action_space = self.get_action_space()
    
    def reset(self):
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        self.plane = p.loadURDF("plane.urdf")
        self.robot = p.loadURDF(r"kinova-ros\kinova_description\urdf\j2s7s300_standalone.urdf", basePosition=[0, 0, 0.1])
        return self.get_observation()
    
    def step(self, action):
        p.setJointMotorControlArray(self.robot, 
                                    range(len(action)), 
                                    p.TORQUE_CONTROL, 
                                    forces=action)
        p.stepSimulation()
        time.sleep(TIME_DELTA)
        observation = self.get_observation()
        reward = self.calculate_reward(observation)
        done = self.check_done(observation)
        return observation, reward, done, {}
    
    def get_observation_space(self):
        return np.zeros(10)  # Example state dimension (customize this)
    
    def get_action_space(self):
        return np.zeros(6)  # Example action dimension (customize this)
    
    def get_observation(self):
        position, _ = p.getBasePositionAndOrientation(self.robot)
        return np.array(position + [0] * 7)  # Customize observation
    
    def calculate_reward(self, observation):
        return 0  # Customize reward calculation
    
    def check_done(self, observation):
        return False  # Customize termination condition
    
    def close(self):
        p.disconnect()

# Main function
def main():
    print("Initializing environment...")
    env = PyBulletEnv()
    print("Environment initialized. Waiting for 10 seconds to ensure everything is set up...")
    time.sleep(10)

    print("Creating agent...")
    agent = Agent(state_dim=env.observation_space.shape[0], action_dim=env.action_space.shape[0])
    
    for i in range(3):
        print(f"Loading trained model for episode {(i+1)*100}...")
        agent.load_models(f'best_model_episode_{(i+1)*100}')
        print("Model loaded successfully!")

        input("Press Enter to start testing...")

        target_positions = [
            np.array([2.0, 2.0, 2.0]),
            np.array([2.0, 2.0, 2.0]),
            np.array([2.0, 2.0, 2.0]),
            np.array([2.0, 2.0, 2.0]),
            np.array([2.0, 2.0, 2.0]),
        ]

        successful_attempts = 0
        total_steps = 0

        for j, target in enumerate(target_positions):
            print(f"\nAttempting to reach target {j+1}: {target}")
            success, steps = test_model(env, agent, target)
            total_steps += steps

            if success:
                successful_attempts += 1
                print(f"Success! Reached the target in {steps} steps.")
            else:
                print(f"Failed to reach the target within the step limit.")
        
        print("\nResults:")
        print(f"Successful attempts: {successful_attempts} out of {len(target_positions)}")
        print(f"Success rate: {successful_attempts / len(target_positions) * 100:.2f}%")
        print(f"Average steps per attempt: {total_steps / len(target_positions):.2f}")

    env.close()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"An error occurred: {e}")
