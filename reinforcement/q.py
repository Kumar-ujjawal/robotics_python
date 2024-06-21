import gym
import matplotlib.pyplot as plt

env = gym.make('MountainCar-v0',render_mode='rgb_array')
env.reset()

done = False

while not done:
    action = 2
    step_output = env.step(action)
    # print(step_output)  # Debug: print the output to see its structure
    
    new_state, reward, done, info,_ = step_output
    env.render()
    


env.close()
