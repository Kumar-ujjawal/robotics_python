import pybullet as p
import pybullet_data
import time
import math
import numpy as np
import random
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.distributions import Normal, MultivariateNormal
import os
import gym
from gym import spaces
from torch.utils.tensorboard import SummaryWriter
import datetime

class KinovaJacoEnv(gym.Env):
    def __init__(self):
        super(KinovaJacoEnv, self).__init__()
        
        # Connect to PyBullet
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        # URDF path for Kinova arm
        urdf_path = r"kinova-ros\kinova_description\urdf\j2s7s300_standalone.urdf"
        
        # Load plane and robot
        self.planeId = p.loadURDF("plane.urdf")
        self.robotId = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=True)
        
        # Initial joint positions
        self.initial_joint_positions = [0.0, 0.0, 2.9, 1.3, -2.07, 1.4, 0.0]
        for i, pos in enumerate(self.initial_joint_positions):
            p.resetJointState(self.robotId, i, pos)
        
        # Get controllable joints
        self.controllable_joints = []
        for i in range(p.getNumJoints(self.robotId)):
            joint_info = p.getJointInfo(self.robotId, i)
            if joint_info[2] != p.JOINT_FIXED:
                self.controllable_joints.append(i)
        
        # Action and observation spaces
        self.action_dim = len(self.controllable_joints)
        self.obs_dim = self.action_dim * 2 + 3 + 1  # joint pos, vel, end effector pos, time
        
        self.action_space = spaces.Box(low=-0.6, high=0.6, shape=(self.action_dim,), dtype=np.float64)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(self.obs_dim,), dtype=np.float64)
        
        # Goal and episode tracking
        self.goal_position = np.zeros(3)
        self.episode_time = 0
        self.current_step = 0
        self.max_step = 1000
    
    def compute_end_effector_position(self):
        # Get end effector link state
        end_effector_state = p.getLinkState(self.robotId, p.getNumJoints(self.robotId) - 1)
        return np.array(end_effector_state[0])
    
    def get_joint_states(self):
        joint_states = p.getJointStates(self.robotId, self.controllable_joints)
        positions = [state[0] for state in joint_states]
        velocities = [state[1] for state in joint_states]
        return positions, velocities
    
    def calculate_reward(self, distance, action):
        # Distance to goal
        distance_reward = -np.exp(0.2 * distance)
        
        # Action smoothness
        action_smoothness = -0.1 * np.sum(np.square(action))
        
        # Energy efficiency
        energy_penalty = -0.01 * np.sum(np.square(action))
        
        # Combine rewards
        reward = (
            2 * distance_reward +
            0.1 * action_smoothness +
            0.2 * energy_penalty
        )
        
        # Bonus for reaching the goal
        if distance < 1.25:
            reward += 50
        if distance < 0.85:
            reward += 100
        if distance < 0.5:
            reward += 200
        if distance < 0.1:
            reward += 1000
        
        return reward
    
    def step(self, action):
        # Apply joint velocities
        for i, joint in enumerate(self.controllable_joints):
            p.setJointMotorControl2(
                bodyUniqueId=self.robotId,
                jointIndex=joint,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=action[i],
                force=1000.0
            )
        
        # Step simulation
        p.stepSimulation()
        time.sleep(1.0 / 240.0)
        
        # Update states
        positions, velocities = self.get_joint_states()
        end_effector_position = self.compute_end_effector_position()
        
        self.episode_time += 1.0 / 240.0
        self.current_step += 1
        
        # Compute next state
        next_state = np.concatenate([
            positions, 
            velocities, 
            end_effector_position, 
            [self.episode_time]
        ])
        
        # Compute distance to goal
        distance_to_goal = np.linalg.norm(end_effector_position - self.goal_position)
        
        # Compute reward
        reward = self.calculate_reward(distance_to_goal, action)
        
        # Check if done
        done = (distance_to_goal < 0.5) or (self.current_step >= self.max_step)
        
        return next_state, reward, done, {}
    
    def reset(self):
        # Reset joint positions
        for i, pos in enumerate(self.initial_joint_positions):
            p.resetJointState(self.robotId, i, pos)
        
        # Reset goal position
        self.goal_position = np.random.uniform(low=-3, high=3, size=3)
        self.goal_position[2] = abs(self.goal_position[2])
        
        # Reset tracking variables
        self.episode_time = 0
        self.current_step = 0
        
        # Get initial state
        positions, velocities = self.get_joint_states()
        end_effector_position = self.compute_end_effector_position()
        
        return np.concatenate([
            positions, 
            velocities, 
            end_effector_position, 
            [self.episode_time]
        ])
    
    def close(self):
        p.disconnect()

# Actor and Critic Networks (same as ROS implementation)
class ActorNetwork(nn.Module):
    def __init__(self, n_actions, state_dim, fc1_dims=256, fc2_dims=256):
        super(ActorNetwork, self).__init__()
        self.fc1 = nn.Linear(state_dim, fc1_dims).to(torch.float32)
        self.bn1 = nn.BatchNorm1d(fc1_dims)
        
        self.fc2 = nn.Linear(fc1_dims, fc2_dims).to(torch.float32)
        self.bn2 = nn.BatchNorm1d(fc2_dims)
        
        self.mean = nn.Linear(fc2_dims, n_actions).to(torch.float32)
        self.log_std = nn.Parameter(torch.zeros(n_actions, dtype=torch.float32))
        
        self.apply(self._init_weights)
        self.disable_bn()
    
    def _init_weights(self, module):
        if isinstance(module, nn.Linear):
            nn.init.orthogonal_(module.weight, gain=np.sqrt(2))
            module.bias.data.zero_()
    
    def forward(self, state):
        x = torch.relu(self.bn1(self.fc1(state)))
        x = torch.relu(self.bn2(self.fc2(x)))
        mean = self.mean(x)
        std = self.log_std.exp()
        return mean, std
    
    def disable_bn(self):
        self.bn1.eval()
        self.bn2.eval()

class CriticNetwork(nn.Module):
    def __init__(self, state_dim, fc1_dims=256, fc2_dims=256):
        super(CriticNetwork, self).__init__()
        self.fc1 = nn.Linear(state_dim, fc1_dims).to(torch.float32)
        self.fc2 = nn.Linear(fc1_dims, fc2_dims).to(torch.float32)
        self.value = nn.Linear(fc2_dims, 1).to(torch.float32)
        
        self.apply(self._init_weights)
    
    def _init_weights(self, module):
        if isinstance(module, nn.Linear):
            nn.init.orthogonal_(module.weight, gain=np.sqrt(2))
            module.bias.data.zero_()
    
    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        value = self.value(x)
        return value

class Agent:
    def __init__(self, state_dim, action_dim, lr=1e-4, gamma=0.99, eps_clip=0.2, 
                 epochs=30, batch_size=32, value_loss_coef=0.5, entropy_coef=0.01):
        self.actor = ActorNetwork(action_dim, state_dim)
        self.critic = CriticNetwork(state_dim)
        
        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=lr)
        self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=lr)
        
        self.gamma = gamma
        self.eps_clip = eps_clip
        self.epochs = epochs
        self.batch_size = batch_size
        self.value_loss_coef = value_loss_coef
        self.entropy_coef = entropy_coef
    
    def select_action(self, state):
        with torch.no_grad():
            state = torch.tensor(state, dtype=torch.float32).unsqueeze(0)
            mean, std = self.actor(state)
        
        mean = torch.nan_to_num(mean, nan=0.0)
        std = torch.nan_to_num(std, nan=1.0)
        
        cov_matrix = torch.diag(std**2)
        dist = MultivariateNormal(mean, covariance_matrix=cov_matrix)
        
        action = dist.sample()
        action = torch.tanh(action) * 0.6  # Scale to action space
        action_log_prob = dist.log_prob(action)
        
        return action.numpy()[0], action_log_prob.detach()
    
    def learn(self, trajectories):
        states, actions, log_probs, rewards, next_states, dones = zip(*trajectories)
        
        states = torch.tensor(states, dtype=torch.float32)
        actions = torch.tensor(actions, dtype=torch.float32)
        old_log_probs = torch.stack(log_probs)
        rewards = torch.tensor(rewards, dtype=torch.float32)
        next_states = torch.tensor(next_states, dtype=torch.float32)
        dones = torch.tensor(dones, dtype=torch.bool)
        
        with torch.no_grad():
            values = self.critic(states).squeeze()
            next_values = self.critic(next_states).squeeze()
            advantages = self._compute_advantages(
                rewards.numpy(), 
                values.numpy(), 
                next_values.numpy(), 
                dones.numpy()
            )
            
            advantages = torch.tensor(advantages, dtype=torch.float32)
            advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
            returns = advantages + values
        
        for _ in range(self.epochs):
            for i in range(0, len(states), self.batch_size):
                batch_indices = slice(i, i + self.batch_size)
                batch_states = states[batch_indices]
                batch_actions = actions[batch_indices]
                batch_log_probs = old_log_probs[batch_indices]
                batch_returns = returns[batch_indices]
                batch_advantages = advantages[batch_indices]
                
                mean, std = self.actor(batch_states)
                dist = Normal(mean, std)
                new_log_probs = dist.log_prob(batch_actions).sum(dim=-1)
                
                log_ratio = new_log_probs - batch_log_probs
                log_ratio = torch.clamp(log_ratio, -20, 20)
                
                ratios = torch.exp(log_ratio)
                
                surr1 = ratios * batch_advantages
                surr2 = torch.clamp(ratios, 1.0 - self.eps_clip, 1.0 + self.eps_clip) * batch_advantages
                
                policy_loss = -torch.min(surr1, surr2).mean()
                entropy = dist.entropy().mean()
                
                value_pred = self.critic(batch_states).squeeze()
                value_loss = F.mse_loss(value_pred, batch_returns)
                
                total_loss = policy_loss + self.value_loss_coef * value_loss - self.entropy_coef * entropy
                
                # Update critic
                self.critic_optimizer.zero_grad()
                value_loss.backward()
                self.critic_optimizer.step()
                
                # Update actor
                self.actor_optimizer.zero_grad()
                policy_loss.backward()
                self.actor_optimizer.step()
    
    def _compute_advantages(self, rewards, values, next_values, dones, gamma=0.99, lmbda=0.95):
        advantages = []
        gae = 0
        for step in reversed(range(len(rewards))):
            delta = rewards[step] + gamma * next_values[step] * (1 - dones[step]) - values[step]
            gae = delta + gamma * lmbda * (1 - dones[step]) * gae
            advantages.insert(0, gae)
        return advantages
    
    def save_models(self, path='models'):
        os.makedirs(path, exist_ok=True)
        torch.save(self.actor.state_dict(), os.path.join(path, 'actor.pth'))
        torch.save(self.critic.state_dict(), os.path.join(path, 'critic.pth'))
    
    def load_models(self, path='models'):
        self.actor.load_state_dict(torch.load(os.path.join(path, 'actor.pth')))
        self.critic.load_state_dict(torch.load(os.path.join(path, 'critic.pth')))

def evaluate(agent, env, num_episodes=5, max_steps=500):
    total_rewards = []
    episode_lengths = []
    
    for _ in range(num_episodes):
        state = env.reset()
        episode_reward = 0
        done = False
        steps = 0
        
        while not done and steps < max_steps:
            action, _ = agent.select_action(state)
            state, reward, done, _ = env.step(action)
            episode_reward += reward
            steps += 1
        
        total_rewards.append(episode_reward)
        episode_lengths.append(steps)
    
    return np.mean(total_rewards), np.mean(episode_lengths)

def train(max_episodes=1000, log_interval=10):
    # Create environment and agent
    env = KinovaJacoEnv()
    state_dim = env.observation_space.shape[0]
    action_dim = env.action_space.shape[0]
    agent = Agent(state_dim, action_dim)
    
    # Tensorboard logging
    writer = SummaryWriter(log_dir=f'runs/ppo_kinova_{datetime.datetime.now().strftime("%Y%m%d_%H%M%S")}')
    
    # Training loop
    for episode in range(max_episodes):
        state = env.reset()
        done = False
        episode_reward = 0
        trajectories = []
        
        while not done:
            # Collect trajectories
            action, log_prob = agent.select_action(state)
            next_state, reward, done, _ = env.step(action)
            
            trajectories.append((state, action, log_prob, reward, next_state, done))
            
            state = next_state
            episode_reward += reward
        
        # Learn from collected trajectories
        agent.learn(trajectories)
        
        # Log episode stats
        if episode % log_interval == 0:
            avg_reward, avg_length = evaluate(agent, env)
            writer.add_scalar('Reward/Episode', episode_reward, episode)
            writer.add_scalar('Eval/AverageReward', avg_reward, episode)
            writer.add_scalar('Eval/AverageEpisodeLength', avg_length, episode)
            
            print(f'Episode {episode}: Reward = {episode_reward}, '
                  f'Avg Eval Reward = {avg_reward}, '
                  f'Avg Eval Length = {avg_length}')
        
        # Periodically save models
        if episode % 100 == 0:
            agent.save_models()
    
    # Close environment and writer
    env.close()
    writer.close()

# Main execution
if __name__ == '__main__':
    train()