import pybullet as p
import pybullet_data
import math
import time
from scipy.spatial.transform import Rotation as R

def anchor(skinId, boneId, tmpIdx = 0):
    p.createSoftBodyAnchor(skinId, 0, boneId, 0+tmpIdx)
    p.createSoftBodyAnchor(skinId, 1, boneId, 0+tmpIdx)
    p.createSoftBodyAnchor(skinId, 4, boneId, 0+tmpIdx)
    p.createSoftBodyAnchor(skinId, 5, boneId, 0+tmpIdx)

    p.createSoftBodyAnchor(skinId, 32, boneId, 1+tmpIdx)
    p.createSoftBodyAnchor(skinId, 33, boneId, 1+tmpIdx)
    p.createSoftBodyAnchor(skinId, 35, boneId, 1+tmpIdx)
    p.createSoftBodyAnchor(skinId, 34, boneId, 1+tmpIdx)

    p.createSoftBodyAnchor(skinId, 24, boneId, 2+tmpIdx)
    p.createSoftBodyAnchor(skinId, 25, boneId, 2+tmpIdx)
    p.createSoftBodyAnchor(skinId, 26, boneId, 2+tmpIdx)
    p.createSoftBodyAnchor(skinId, 27, boneId, 2+tmpIdx)

    p.createSoftBodyAnchor(skinId, 28, boneId, 3+tmpIdx)
    p.createSoftBodyAnchor(skinId, 29, boneId, 3+tmpIdx)
    p.createSoftBodyAnchor(skinId, 30, boneId, 3+tmpIdx)
    p.createSoftBodyAnchor(skinId, 31, boneId, 3+tmpIdx)

    p.createSoftBodyAnchor(skinId, 8, boneId, 4+tmpIdx)
    p.createSoftBodyAnchor(skinId, 9, boneId, 4+tmpIdx)
    p.createSoftBodyAnchor(skinId, 10, boneId, 4+tmpIdx)
    p.createSoftBodyAnchor(skinId, 11, boneId, 4+tmpIdx)

    p.createSoftBodyAnchor(skinId, 16, boneId, 5+tmpIdx)
    p.createSoftBodyAnchor(skinId, 17, boneId, 5+tmpIdx)
    p.createSoftBodyAnchor(skinId, 18, boneId, 5+tmpIdx)
    p.createSoftBodyAnchor(skinId, 19, boneId, 5+tmpIdx)

    p.createSoftBodyAnchor(skinId, 12, boneId, 6+tmpIdx)
    p.createSoftBodyAnchor(skinId, 13, boneId, 6+tmpIdx)
    p.createSoftBodyAnchor(skinId, 14, boneId, 6+tmpIdx)
    p.createSoftBodyAnchor(skinId, 15, boneId, 6+tmpIdx)

    p.createSoftBodyAnchor(skinId, 20, boneId, 7+tmpIdx)
    p.createSoftBodyAnchor(skinId, 21, boneId, 7+tmpIdx)
    p.createSoftBodyAnchor(skinId, 22, boneId, 7+tmpIdx)
    p.createSoftBodyAnchor(skinId, 23, boneId, 7+tmpIdx)

    p.createSoftBodyAnchor(skinId, 2, boneId, 8+tmpIdx)
    p.createSoftBodyAnchor(skinId, 3, boneId, 8+tmpIdx)
    p.createSoftBodyAnchor(skinId, 6, boneId, 8+tmpIdx)
    p.createSoftBodyAnchor(skinId, 7, boneId, 8+tmpIdx)

physicsClient = p.connect(p.GUI) # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally : register the directory
p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD) # enable FEM for deformable things

# load a plane
planeId = p.loadURDF("plane.urdf")
#plane = p.createCollisionShape(p.GEOM_PLANE)
# p.createMultiBody(0, plane)

p.setGravity(0,0,-8)


""" CREATE BONES """

# create a cylinder collision shape
cylinderRadius = 0.1
cylinderHeight = 0.3
gap = 0.2
cylinderColPos = [0,0,cylinderHeight/2]
cylinderColOri = p.getQuaternionFromEuler([0,0,1])
cylinderId = p.createCollisionShape(p.GEOM_CYLINDER,radius = cylinderRadius, height = cylinderHeight)#, collisionFramePosition = cylinderColPos)#, collisionFrameOrientation = cylinderColOri)
# to make it static
# boxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=[cylinderRadius, cylinderRadius, cylinderRadius])

# link
linkMasses = []
linkCollisionShapeIndices = []
linkVisualShapeIndices = []
linkPositions = []
linkOrientations = []
linkInertialFramePositions = []
linkInertialFrameOrientations = []
linkParentIndices = []
linkJointTypes = []
linkJointAxis = []

baseMass = 1 # 0 : static
basePosition = [0,0,2]#cylinderRadius]
baseOrientation = [1,0,0,1] # rotate 2*theta from a vector (x, y, z). w = cos(theta)
# baseOrientation = p.getQuaternionFromEuler([0, math.pi/2, 0]) # Euler angles

for i in range(9):
    linkMasses.append(baseMass)
    linkCollisionShapeIndices.append(cylinderId)
    linkVisualShapeIndices.append(-1)
    linkPositions.append([0, 0, cylinderHeight + gap])
    linkOrientations.append([0,0,0,1])
    linkInertialFramePositions.append([0,0,0])
    linkInertialFrameOrientations.append([0,0,0,1])
    linkParentIndices.append(i)
    linkJointTypes.append(p.JOINT_SPHERICAL)
    linkJointAxis.append([0,1,0])

# create rigid objects by shapes
snakeId = p.createMultiBody(baseMass, cylinderId, -1, basePosition, baseOrientation, 
                                linkMasses=linkMasses,
                              linkCollisionShapeIndices=linkCollisionShapeIndices,
                              linkVisualShapeIndices=linkVisualShapeIndices,
                              linkPositions=linkPositions,
                              linkOrientations=linkOrientations,
                              linkInertialFramePositions=linkInertialFramePositions,
                              linkInertialFrameOrientations=linkInertialFrameOrientations,
                              linkParentIndices=linkParentIndices,
                              linkJointTypes=linkJointTypes,
                              linkJointAxis=linkJointAxis)
                              # useMaximalCoordinates=True) # seperate links

            
soft = True
numJoints = p.getNumJoints(snakeId)

""" load soft body """
if soft:
    # Available files : cube.obj, cube2.obj, sphere_smooth.obj, duck.obj 
    # p.loadSoftBody("cube2.obj", basePosition = basePosition , scale = 1, mass = 1., useNeoHookean = 0, useBendingSprings=1,useMassSpring=1, springElasticStiffness=40, springDampingStiffness=.1, springDampingAllDirections = 1, useSelfCollision = 0, frictionCoeff = .5, useFaceContact=1, collisionMargin = 0.04)
    print("Start loading...")
    clothId = p.loadSoftBody("soft_cube_snake.obj", basePosition = [0,-2.5,2], scale = 0.5, mass = 0, useNeoHookean = 0, useBendingSprings=1,useMassSpring=1, springElasticStiffness=40, springDampingStiffness=.1, springDampingAllDirections = 1, useSelfCollision = 0, frictionCoeff = .5, useFaceContact=True)
    print("Loading is completed...")

    # remove collisions
    for i in range(-1, numJoints):
        p.setCollisionFilterGroupMask(snakeId, i, 0, 0)
        p.setCollisionFilterPair(0, snakeId, -1, i, 1)
        p.setCollisionFilterPair(snakeId, clothId,i, -1, 0)

    anchor(clothId, snakeId)

""" debug """
debug = True
if soft and debug:
    data = p.getMeshData(clothId, -1, flags=p.MESH_DATA_SIMULATION_MESH)
    print("--------------")
    print("data=",data)
    print(data[0])
    print(data[1])
    text_uid = []
    for i in range(data[0]):
        pos = data[1][i]
        uid = p.addUserDebugText(str(i+1), pos, textColorRGB=[1,1,1])
        text_uid.append(uid)
      
""" Softbody anchor """
if soft:
    print("numJoints :" + str(numJoints))
    # createSoftBodyAnchor params : "softBodyBodyUniqueId", "nodeIndex", "bodyUniqueId", "linkIndex", "bodyFramePosition", "physicsClientId",
    # vtxNum = 4 # 12
    # i = 0
    # for j in range(nodeNum):
    #     p.createSoftBodyAnchor(clothId, i+j, snakeId, i)
    #     p.createSoftBodyAnchor(clothId, i+j+vtxNum, snakeId, i)
    


""" Movement """
# change dynamics
anistropicFriction = [1, 0.01, 0.01] # move to X dir
p.changeDynamics(snakeId, -1, lateralFriction=2, anisotropicFriction=anistropicFriction)

for i in range(p.getNumJoints(snakeId)):
    p.changeDynamics(snakeId, i, lateralFriction=2, anisotropicFriction=anistropicFriction)

# calculate movements
dt = 1./240.
SNAKE_NORMAL_PERIOD = 1.5
m_wavePeriod = SNAKE_NORMAL_PERIOD
m_waveLength = 4
m_waveAmplitude = 0.8
m_waveFront = 0.0
m_segmentLength = gap + cylinderHeight
# forward = 0
m_steering = 0.0


""" move """
# p.setRealTimeSimulation(0)
start_time = time.time()
while True:
    keys = p.getKeyboardEvents()
    for k, v in keys.items():

        if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_TRIGGERED)):
            m_steering = -.2
        if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_RELEASED)):
            m_steering = 0
        if (k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_TRIGGERED)):
            m_steering = .2
        if (k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_RELEASED)):
            m_steering = 0

    # amp = 0.2
    # offset = 0.6
    # numMuscles = p.getNumJoints(sphereUid)
    scaleStart = 1.0

    #start of the snake with smaller waves.
    #I think starting the wave at the tail would work better ( while it still goes from head to tail )
    if (m_waveFront < m_segmentLength * 4.0):
        scaleStart = m_waveFront / (m_segmentLength * 4.0)
    
    # segment = numMuscles - 1

    #we simply move a sin wave down the body of the snake.
    #this snake may be going backwards, but who can tell ;)
    numJoint = p.getNumJoints(snakeId)
    for joint in range(numJoint-1, -1, -1):
        segment = joint
        #map segment to phase
        phase = (m_waveFront - (segment + 1) * m_segmentLength) / m_waveLength
        phase -= math.floor(phase)
        phase *= math.pi * 2.0

        #map phase to curvature(곡률)
        targetPos = math.sin(phase) * scaleStart * m_waveAmplitude
        # targetPos = [0.,math.sin(phase) * scaleStart * m_waveAmplitude, 0.]
        # idx = 1

        #// steer snake by squashing +ve or -ve side of sin curve
        if (m_steering > 0 and targetPos < 0):
            targetPos *= 1.0 / (1.0 + m_steering)

        if (m_steering < 0 and targetPos > 0):
            targetPos *= 1.0 / (1.0 - m_steering)
        targetPos += m_steering

        # if (m_steering > 0 and targetPos[idx] < 0):
        #   targetPos[idx] *= 1.0 / (1.0 + m_steering)

        # if (m_steering < 0 and targetPos[idx] > 0):
        #   targetPos[idx] *= 1.0 / (1.0 - m_steering)

        # set our motor
        # p.setJointMotorControl2(sphereUid,
        p.setJointMotorControlMultiDof(snakeId,
                                joint,
                                p.POSITION_CONTROL,
                                # targetPosition=targetPos + m_steering,
                                # targetPosition=[0.,0,targetPos,1.],
                                targetPosition=R.as_quat(R.from_euler('xyz', [0.,targetPos, 0.])),
                                force=[30]) # 0이면 velocity motor disable -> 그래도 잘 다님

        print()
        #If you want a wheel to maintain a constant velocity, with a max force you can use:
        # p.setJointMotorControl2(sphereUid, 
        #                         joint, 
        #                         p.VELOCITY_CONTROL,
        #                         targetVelocity = 15,
        #                         force = 20)

        #wave keeps track of where the wave is in time
    m_waveFront += dt / m_wavePeriod * m_waveLength
    p.stepSimulation()

finish_time = time.time()
print("time : " + str(finish_time - start_time))
# p.disconnect()
