anistropicFriction = [1, 0.01, 0.01]
# p.changeDynamics(sphereUid, -1, lateralFriction=2, anisotropicFriction=anistropicFriction)
# p.getNumJoints(sphereUid)
# for i in range(p.getNumJoints(sphereUid)):
#   p.getJointInfo(sphereUid, i)
#   p.changeDynamics(sphereUid, i, lateralFriction=2, anisotropicFriction=anistropicFriction)

# dt = 1. / 240.
# SNAKE_NORMAL_PERIOD = 0.1  #1.5
# m_wavePeriod = SNAKE_NORMAL_PERIOD # 주기

# m_waveLength = 4 # 파동 길이
# m_wavePeriod = 1.5 # 파동 주기
# m_waveAmplitude = 0.4 # 파동 진폭
# m_waveFront = 0.0 # 파동이 주기의 어디 부분부터 시작하는지?
# #our steering value
# m_steering = 0.0 # 얼마나 어느쪽으로 방향 트는지
# m_segmentLength = sphereRadius * 2.0 # 곡선의 segment(구의 길이)
# # forward = 0

# while (True):
#   keys = p.getKeyboardEvents()
#   for k, v in keys.items():

#     if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_TRIGGERED)):
#       m_steering = -.2
#     if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_RELEASED)):
#       m_steering = 0
#     if (k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_TRIGGERED)):
#       m_steering = .2
#     if (k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_RELEASED)):
#       m_steering = 0

#   # amp = 0.2
#   # offset = 0.6
#   # numMuscles = p.getNumJoints(sphereUid)
#   scaleStart = 1.0

#   #start of the snake with smaller waves.
#   #I think starting the wave at the tail would work better ( while it still goes from head to tail )
#   if (m_waveFront < m_segmentLength * 4.0):
#     scaleStart = m_waveFront / (m_segmentLength * 4.0)
  
#   # segment = numMuscles - 1

#   #we simply move a sin wave down the body of the snake.
#   #this snake may be going backwards, but who can tell ;)
#   for joint in range(p.getNumJoints(sphereUid)):
#     segment = joint  #numMuscles-1-joint
#     #map segment to phase
#     phase = (m_waveFront - (segment + 1) * m_segmentLength) / m_waveLength
#     phase -= math.floor(phase)
#     phase *= math.pi * 2.0

#     #map phase to curvature(곡률)
#     targetPos = math.sin(phase) * scaleStart * m_waveAmplitude

#     #// steer snake by squashing +ve or -ve side of sin curve
#     if (m_steering > 0 and targetPos < 0):
#       targetPos *= 1.0 / (1.0 + m_steering)

#     if (m_steering < 0 and targetPos > 0):
#       targetPos *= 1.0 / (1.0 - m_steering)

#     # set our motor
#     p.setJointMotorControl2(sphereUid,
#                             joint,
#                             p.POSITION_CONTROL,
#                             targetPosition=targetPos + m_steering,
#                             force=30) # 0이면 velocity motor disable -> 그래도 잘 다님

#     #If you want a wheel to maintain a constant velocity, with a max force you can use:
#     # p.setJointMotorControl2(sphereUid, 
#     #                         joint, 
#     #                         p.VELOCITY_CONTROL,
#     #                         targetVelocity = 15,
#     #                         force = 20)

#   #wave keeps track of where the wave is in time
#   m_waveFront += dt / m_wavePeriod * m_waveLength
#   p.stepSimulation()

#   time.sleep(dt)
