import numpy as np
from kinova_env import KinovaEnv
from q_learning_agent import QLearningAgent

def train():
    env = KinovaEnv()
    agent = QLearningAgent(action_space=env.action_space, state_space=len(env._get_state()))

    episodes = 1000
    for episode in range(episodes):
        state = env.reset()
        total_reward = 0

        for t in range(6000):
            action = agent.choose_action(state)
            next_state, reward, done = env.step(action)
            agent.update(state, action, reward, next_state)
            state = next_state
            total_reward += reward

            if done:
                break

        print(f"Episode {episode + 1}/{episodes}, Total Reward: {total_reward:.2f}")

    env.close()

if __name__ == "__main__":
    train()