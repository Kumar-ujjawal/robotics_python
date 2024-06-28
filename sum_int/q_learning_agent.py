import numpy as np
import random
from collections import defaultdict

class QLearningAgent:
    def __init__(self, action_space, state_space, lr=0.1, gamma=0.99, epsilon=0.1, state_bins=10, action_bins=5):
        self.action_space = action_space
        self.state_space = state_space
        self.lr = lr
        self.gamma = gamma
        self.epsilon = epsilon
        self.state_bins = state_bins
        self.action_bins = action_bins
        self.q_table = defaultdict(lambda: defaultdict(float))

    def discretize_state(self, state):
        discretized = np.digitize(state, np.linspace(-1, 1, self.state_bins))
        return tuple(discretized)

    def discretize_action(self, action):
        discretized = np.digitize(action, np.linspace(-1, 1, self.action_bins))
        return tuple(discretized)

    def choose_action(self, state):
        if random.uniform(0, 1) < self.epsilon:
            return np.random.uniform(-1, 1, self.action_space)
        
        state = self.discretize_state(state)
        if state not in self.q_table or not self.q_table[state]:
            return np.random.uniform(-1, 1, self.action_space)
        
        best_action = max(self.q_table[state], key=self.q_table[state].get)
        return np.array([np.linspace(-1, 1, self.action_bins)[a] for a in best_action])

    def update(self, state, action, reward, next_state):
        state = self.discretize_state(state)
        next_state = self.discretize_state(next_state)
        action = self.discretize_action(action)
        
        current_q = self.q_table[state][action]
        
        if next_state in self.q_table:
            best_next_q = max(self.q_table[next_state].values()) if self.q_table[next_state] else 0
        else:
            best_next_q = 0
        
        td_target = reward + self.gamma * best_next_q
        td_error = td_target - current_q
        self.q_table[state][action] = current_q + self.lr * td_error