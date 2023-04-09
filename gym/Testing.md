# 테스트할 때 참고사항
## SnakeModel
### createSkin()
1. `loadSoftBody()`의 파라미터에서 `mass = .1` (0 초과)로 설정해야 앞으로 움직임
2. `loadSoftBody()`의 파라미터에서 `useBendingSprings=0, useMassSpring=0` 로 설정해야 몸이 안뜨는 듯 하다
    -> 10만번 학습 시키니 날라다닌다.
    -> `moveSin()`에서 `setJointMotorControlMultiDof()`의 파라미터 `force`(도달할 수 있는 max force) 값을 낮춰서 테스트 해보자.

### moveSin()
1. `p.stepSimulation()`은 for문 밖 아래로 빼야한다. (안그럴 경우 관절을 하나 하나 순차적으로 바꾸기 때문에 학습 매우 오래 걸릴 것)
2. `moveSin()` 파라미터 `dt`와 `force`를 크게 설정할 수록 빠르게 움직인다.
4. `p.setJointMotorControlMultiDof()`의 파라미터 `targetPosition`값이 `[0.,np.sin(phase) * scaleStart * waveAmplitude,0.,1.]` 일 때 sin파로 움직인다.
    -> `p.setJointMotorControlMultiDofArray()` 사용시 파라미터 `targetPositions`값이 `[0.,np.sin(phase) * scaleStart * waveAmplitude,0.,1.]`의 리스트 형태일 때 sin파로 움직인다.
5. 계산 속도를 위해 기존처럼 for문 아래에서 `p.setJointMotorControlMultiDofArray()`를 쓰자. (이것저것 테스트해보느라 최근 커밋('edit softbody anchor')에선 for문 속에 `p.setJointMotorControlMultiDof()`를 사용했다.)
6. (참고) 아래는 사인파대로 움직이는 `moveSin()`함수이다.

'''

    # original moveSin

    def moveSin(self, action, force = 600, dt = 1., wavePeriod = 1.5, waveLength = 4, waveAmplitude = 0.8):
        jointXYZActions = np.zeros(4 * self.jointNum)

        for i in range(self.jointNum):
            jointXYZActions[4*i + 3] = 1.0
            jointXYZActions[4*i + 1] = action[i]
        
        segmentLength = self.gap + self.nodeLength
        scaleStart = 1.0
        waveFront = self.waveFront

        # start waves
        if (waveFront < segmentLength * 4.0):
            scaleStart = waveFront / (segmentLength * 4.0)

        for joint in range(self.jointNum-1,-1,-1):
            segment = joint

            # map segment to phase
            phase = (waveFront - (segment + 1) * segmentLength) / waveLength
            phase -= np.floor(phase)
            phase *= np.pi * 2.0

            # map phase to curvature
            pos = np.sin(phase) * scaleStart * waveAmplitude

            # steering = jointXYZActions[joint*4+1]
            # if (steering > 0 and pos < 0):
            #     pos *= 1.0 / (1.0 + steering)

            # if (steering < 0 and pos > 0):
            #     pos *= 1.0 / (1.0 - steering)

            jointXYZActions[joint*4+1] = pos

        targetPos = np.reshape(jointXYZActions,(self.jointNum, 4)).tolist()
        forces = np.full(self.jointNum*3,force, dtype=np.float32)
        forces = np.reshape(forces, (self.jointNum,3)).tolist()

        p.setJointMotorControlMultiDofArray(self.boneId,
                                    self.jointIndices,
                                    p.POSITION_CONTROL,
                                    targetPositions=targetPos,
                                    forces=forces)

        self.waveFront += dt / wavePeriod * waveLength
        p.stepSimulation()
        self.updateStates()

'''
