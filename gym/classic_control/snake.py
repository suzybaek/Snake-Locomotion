import pybullet as p
import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import random
from gym.envs.classic_control.SnakeModel import *

class SnakeEnv(gym.Env):

    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second': 50
    }

    def __init__(self):
        snakeLength = 10
        self.snake = Snake(snakeLength)
        self.forces = np.full((self.snake.jointNum, 3), 500, dtype=np.float32)

        # init STATE, ACTION SPACE
        self.state = None
        self.goal = []
        self.goalOffset = 15

        threshold = 300
        threshold_low = np.concatenate((np.full((self.snake.jointNum+1)*6,-threshold), [-self.goalOffset, -self.goalOffset]))# coordinates + velocities
        threshold_high = np.concatenate((np.full((self.snake.jointNum+1)*6, threshold), [self.goalOffset, self.goalOffset]))

        maxSteering = 0.2
        act_low = np.array([-maxSteering])
        act_high = np.array([maxSteering])

        self.action_space = spaces.Box(low=act_low, high=act_high)
        self.observation_space = spaces.Box(low= threshold_low, high= threshold_high)

        self.debug = None
        self.debug1 = None

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        past_head = self.snake.headWorldPos
        past_state = self.state

        self.snake.moveSin(steering = action[0],
                            dt = 1./24., forces = self.forces, wavePeriod = 1.0, waveLength = 8,
                            waveAmplitude = 0.9)

        localGoal = self.snake.getLocalCoordinate(self.goal - self.snake.midWorldPos) # pos으로 계산
        self.state = np.concatenate((self.snake.linksLocalPositionAndVelocity, localGoal[:2]))

        # reward_1 : head local dir 벡터와 local head to local goal벡터 사이 각을 최소화
        headAheadVec = self.snake.headLocalAheadDir
        headToGoalUnitVec = np.array(localGoal) - np.array(self.state[:3])
        headToGoalUnitVec /= np.linalg.norm(headToGoalUnitVec)
        reward_1 = np.dot(headToGoalUnitVec, headAheadVec)

        # reward_2 : goal local벡터와 몸통의 y축(tail to head) 각도를 최소화
        # unitMidToGoal = self.goal - self.snake.midWorldPos #world 좌표
        unitMidToGoal = localGoal
        # unitMidToGoal /= np.linalg.norm(unitMidToGoal)
        tailToHead = self.snake.midWorldOri4M[:3,:3] @ [0.,1.,0.]
        # tailToHead = self.snake.headWorldPos - self.snake.tailWorldPos #world 좌표
        tailToHead /= np.linalg.norm(tailToHead)
        reward_2 = np.dot(unitMidToGoal, tailToHead)

        # reward_3 : 몸통의 속도를 최대화
        reward_3 = np.linalg.norm(self.snake.totalLocalVel)

        # reward_4 : goal까지의 거리를 최소화(중앙노드 -> head node)
        reward_4 = (1 / (np.linalg.norm(self.state[:3] - localGoal) + 1e-6))
        # reward_4 = (1 / (np.linalg.norm(self.snake.headWorldPos - self.goal) + 1e-6))

        # reward_5 : torque minimization / 1e+6 단위
        reward_5 = 1 / (self.snake.getJointsTorqueSum() + 1e-6)

        # reward_6 : acceleration 최소 / 100단위
        velocities = self.state[3*self.snake.nodeNum:]
        past_velocities = past_state[3*self.snake.nodeNum:]
        reward_6 = 1 / (np.sum(np.abs(velocities - past_velocities)) + 1e-6)
        
        # print(reward_1, reward_2, reward_3, reward_4, reward_5, reward_6)

        # 가중치
        w1 = 20#*(1e-5)  #1e+7
        w2 = 20  #10
        w3 = 12  #1
        w4 = 100   #1~2
        w5 = 10000 #44
        w6 = 10 #0.79

        reward = 0.01*(w1*reward_1 + w2*reward_2 + w3*reward_3 + w4*reward_4 + w5*reward_5 + w6*reward_6)

        done = 0

        # debug
        if self.debug is not None:
            p.removeUserDebugItem(self.debug)
            p.removeUserDebugItem(self.debug1)
            p.removeUserDebugItem(self.debug2)
        self.debug = p.addUserDebugLine(self.snake.midWorldPos, (self.snake.midWorldTrans4M @ np.concatenate((localGoal, [1.])))[:3], lineColorRGB = [1,0,0])
        # self.debug = p.addUserDebugLine(self.snake.midWorldPos, self.goal, lineColorRGB = [1,0,0])
        self.debug1 = p.addUserDebugLine(self.snake.midWorldPos, self.snake.midWorldPos + (self.snake.midWorldTrans4M @ np.concatenate((self.snake.totalLocalVel, [0.])))[:3], lineColorRGB = [0,1,1])
        self.debug2 = p.addUserDebugLine(self.snake.headWorldPos, (self.snake.midWorldTrans4M @ np.concatenate((localGoal, [1.])))[:3], lineColorRGB = [0,0,1])
        # self.debug2 = p.addUserDebugLine(self.snake.headWorldPos, self.goal, lineColorRGB = [0,0,1])
        
        # draw trajectory of head
        p.addUserDebugLine(self.snake.headWorldPos,past_head,lineColorRGB=[1,0,1])

        # goal 재설정
        prob = 0.001
        if self.isInGoal() or random.random() < prob:
            self.goalBox.remove()
            self.goal, self.goalBox = self.resetGoal()

        return self.state,reward,done,{}

    def reset(self):
        self.snake.init(soft=True)
        self.goal, self.goalBox = self.resetGoal()
        localGoal = self.snake.getLocalCoordinate(self.goal)
        self.state = np.concatenate((self.snake.linksLocalPositionAndVelocity, [localGoal[0], localGoal[1]]))
        return self.state # must be np.array

    def isInGoal(self):
        for i in range(3):
            if self.goal[i] - 0.5 > self.snake.headWorldPos[i] or self.goal[i] + 0.5 < self.snake.headWorldPos[i]:
                return False
        return True

    def resetGoal(self, base = [0.,0.,0.,], offset = 15):
        worldGoal = [base[0] + random.uniform(-offset,offset), base[1] + random.uniform(-offset, offset), 0.]
        goalBox = BoxNode(worldGoal, 0.2)
        return worldGoal, goalBox

    def getTheta(self, vecA, vecB):
        # 제 2 cos
        vecC = vecB - vecA

        norm_A = np.linalg.norm(vecA)
        norm_B = np.linalg.norm(vecB)
        norm_C = np.linalg.norm(vecC)
        
        cos = (norm_A ** 2 + norm_B ** 2 - norm_C ** 2) / (2 * norm_A * norm_B)
        return np.arccos(cos)
