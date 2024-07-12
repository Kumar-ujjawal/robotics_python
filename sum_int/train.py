import numpy as np
from kinova_env import KinovaEnv
from q_learning_agent import QLearningAgent

def train():
    env = KinovaEnv()
    agent = QLearningAgent(action_space=env.action_space, state_space=len(env._get_state()))

    episodes = 5000
    max_steps = 1000
    
    for episode in range(episodes):
        state = env.reset()
        total_reward = 0

        for t in range(max_steps):
            action = agent.choose_action(state)
            next_state, reward, done = env.step(action)
            agent.update(state, action, reward, next_state)
            state = next_state
            total_reward += reward

            if done:
                break

        # Update curriculum and decay epsilon
        env.update_curriculum()
        agent.decay_epsilon()

        print(f"Episode {episode + 1}/{episodes}, Total Reward: {total_reward:.2f}, Epsilon: {agent.epsilon:.4f}")

        # Save the Q-table periodically
        if (episode + 1) % 100 == 0:
            np.save(f'q_table_episode_{episode + 1}.npy', dict(agent.q_table))

    env.close()

if __name__ == "__main__":
    train()