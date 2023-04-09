import gym
import pybullet as p
import pybullet_data
from stable_baselines.common.policies import MlpPolicy
from stable_baselines import PPO2

# multiprocess environment
env = gym.make('Snake-v0')

learn = False

if learn:
    model = PPO2(MlpPolicy, env,learning_rate=1e-3,verbose=1)

    print("start learning")

    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    model.learn(total_timesteps=10000)

    p.disconnect()

    # model.save("result_model/new4")

# Enjoy trained agent
print("start testing")

obs = env.reset()
# model = PPO2.load("result_model/new2")

score = 0
test_vel = []

while True:
    # print('test..')
    action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)
    score += rewards
    test_vel.append(obs[31]) #base y axis velocity
    # print('test')
    # env.render()
    # print(env.goal)

print('test total rewards: ',score)
