import gym
import pybullet as p
import pybullet_data
from stable_baselines.common.policies import MlpPolicy
from stable_baselines import PPO2

# multiprocess environment
env = gym.make('Snake-v0')

model = PPO2(MlpPolicy, env,learning_rate=1e-3,verbose=1)

#model = PPO2.load("ppo2_snake")

print("start learning")

p.connect(p.DIRECT) # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())

model.learn(total_timesteps=10000)

p.disconnect()

#model.save("ppo2_snake")

# Enjoy trained agent
print("start testing")

obs = env.reset()

score = 0
test_vel = []

for i in range(30000):
    action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)
    score += rewards
    test_vel.append(obs[31]) #base y axis velocity

print('test total rewards: ',score)