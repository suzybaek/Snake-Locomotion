import pybullet as p
import pybullet_data
import numpy as np
from scipy.spatial.transform import Rotation as R

class BoxNode(): # goal box
    def __init__(self, worldPos, halfHeight = 0.5):
        worldPos[2] = halfHeight
        shapeId = p.createVisualShape(p.GEOM_BOX, halfExtents=[halfHeight, halfHeight, halfHeight], rgbaColor = [1.,0.,0.,1.])
        self.id = p.createMultiBody(baseMass = 0, baseVisualShapeIndex = shapeId, basePosition = worldPos)
        self.debugId = p.addUserDebugText('worldGoal', worldPos, [1,0,0])

    def remove(self):
        p.removeUserDebugItem(self.debugId)
        p.removeBody(self.id)

class CylinderNode():
    def __init__(self, radius = 0.2, height = 0.3):
        self.radius = radius
        self.height = height
        self.colPos = [0,0,self.height/2]
        self.colOri = p.getQuaternionFromEuler([0,0,1])
        self.id = p.createCollisionShape(p.GEOM_CYLINDER, radius=self.radius,
                                            height=self.height)
 
class Snake():
    def __init__(self, nodeNum=10, nodeLength = 0.3, gap = 0.2):
        self.radius = 0.1
        self.nodeNum = nodeNum
        self.nodeLength = nodeLength
        self.jointNum = self.nodeNum - 1
        self.gap = gap

        self.jointIndices = np.arange(0, self.jointNum, 1).tolist()
        self.linkIndices = np.arange(0, self.jointNum, 1).tolist() # base는 별도

    def init(self, basePosition = [0,0,0.5], baseOrientation = [np.pi/2,0,0], baseMass = 1, soft = True):
        if not p.isConnected():
            self.physicsClient = p.connect(p.GUI) # or p.DIRECT for non-graphical version
            p.setAdditionalSearchPath(pybullet_data.getDataPath())

        p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
        p.loadURDF("plane.urdf")
        p.setGravity(0, 0, -10)

        self.nodeId = CylinderNode(height=self.nodeLength).id
        self.boneId = self.createBones(baseMass, basePosition, p.getQuaternionFromEuler(baseOrientation))

        if soft:
            skinId1 = self.createSkin("soft_cube_snake.obj", basePosition = [0,-2.3,0.5], scale = 0.5)
            self.anchor(skinId1)

            # when use 20 nodes
            # skinId2 = self.createSkin("soft_cube_snake.obj", basePosition = [0,-7.3,0.5], scale = 0.5)
            # self.anchor(skinId2, 10)
            # self.connectTwoSkins(skinId1, skinId2)

        self.changeDynamics()
        self.waveFront = 0.
        self.updateStates()        

    def createBones(self, baseMass, basePosition, baseQuaternionOrientation):
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
        linkJointAxis = []

        for i in range(self.jointNum):
            linkMasses.append(baseMass) # 0 makes link static
            linkCollisionShapeIndices.append(self.nodeId)
            linkVisualShapeIndices.append(-1)
            linkPositions.append([0, 0, self.nodeLength + self.gap])
            linkOrientations.append([0, 0, 0, 1])
            linkInertialFramePositions.append([0, 0, 0])
            linkInertialFrameOrientations.append([0, 0, 0, 1])
            linkParentIndices.append(i)
            linkJointTypes.append(p.JOINT_SPHERICAL)
            linkJointAxis.append([0, 1, 0])

        # create rigid objects by shapes
        boneId = p.createMultiBody(baseMass, self.nodeId, -1, basePosition, baseQuaternionOrientation,
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

        # remove collisions
        for i in range(-1, self.jointNum):
            p.setCollisionFilterGroupMask(boneId, i, 0, 0)
            p.setCollisionFilterPair(0, boneId, -1, i, 1)
        
        return boneId
    
    def createSkin(self, fileName, basePosition = [0,0,0], baseOrientation = [0,0,0,1], scale = 1.):
        skinId = p.loadSoftBody(fileName, basePosition=basePosition, baseOrientation = baseOrientation, scale=scale, mass=0,
                                    useNeoHookean=0, useBendingSprings=0, useMassSpring=0, springElasticStiffness=10,
                                    springDampingStiffness= .1, springDampingAllDirections=1, useSelfCollision=0,
                                    frictionCoeff=.5, useFaceContact=0)
        
        # remove collisions
        for i in range(-1, self.jointNum):
            p.setCollisionFilterPair(self.boneId, skinId, i, -1, 0)
        
        return skinId
    
    def connectTwoSkins(self, id1, id2):
        p.createSoftBodyAnchor(id1, 2, id2, 0)
        p.createSoftBodyAnchor(id1, 3, id2, 1)
        p.createSoftBodyAnchor(id1, 6, id2, 4)
        p.createSoftBodyAnchor(id1, 7, id2, 5)

    def anchor(self, skinId, tmpIdx = 0):
        p.createSoftBodyAnchor(skinId, 0, self.boneId, 0+tmpIdx)
        p.createSoftBodyAnchor(skinId, 1, self.boneId, 0+tmpIdx)
        p.createSoftBodyAnchor(skinId, 4, self.boneId, 0+tmpIdx)
        p.createSoftBodyAnchor(skinId, 5, self.boneId, 0+tmpIdx)

        p.createSoftBodyAnchor(skinId, 32, self.boneId, 1+tmpIdx)
        p.createSoftBodyAnchor(skinId, 33, self.boneId, 1+tmpIdx)
        p.createSoftBodyAnchor(skinId, 35, self.boneId, 1+tmpIdx)
        p.createSoftBodyAnchor(skinId, 34, self.boneId, 1+tmpIdx)

        p.createSoftBodyAnchor(skinId, 24, self.boneId, 2+tmpIdx)
        p.createSoftBodyAnchor(skinId, 25, self.boneId, 2+tmpIdx)
        p.createSoftBodyAnchor(skinId, 26, self.boneId, 2+tmpIdx)
        p.createSoftBodyAnchor(skinId, 27, self.boneId, 2+tmpIdx)

        p.createSoftBodyAnchor(skinId, 28, self.boneId, 3+tmpIdx)
        p.createSoftBodyAnchor(skinId, 29, self.boneId, 3+tmpIdx)
        p.createSoftBodyAnchor(skinId, 30, self.boneId, 3+tmpIdx)
        p.createSoftBodyAnchor(skinId, 31, self.boneId, 3+tmpIdx)

        p.createSoftBodyAnchor(skinId, 8, self.boneId, 4+tmpIdx)
        p.createSoftBodyAnchor(skinId, 9, self.boneId, 4+tmpIdx)
        p.createSoftBodyAnchor(skinId, 10, self.boneId, 4+tmpIdx)
        p.createSoftBodyAnchor(skinId, 11, self.boneId, 4+tmpIdx)

        p.createSoftBodyAnchor(skinId, 16, self.boneId, 5+tmpIdx)
        p.createSoftBodyAnchor(skinId, 17, self.boneId, 5+tmpIdx)
        p.createSoftBodyAnchor(skinId, 18, self.boneId, 5+tmpIdx)
        p.createSoftBodyAnchor(skinId, 19, self.boneId, 5+tmpIdx)

        p.createSoftBodyAnchor(skinId, 12, self.boneId, 6+tmpIdx)
        p.createSoftBodyAnchor(skinId, 13, self.boneId, 6+tmpIdx)
        p.createSoftBodyAnchor(skinId, 14, self.boneId, 6+tmpIdx)
        p.createSoftBodyAnchor(skinId, 15, self.boneId, 6+tmpIdx)

        p.createSoftBodyAnchor(skinId, 20, self.boneId, 7+tmpIdx)
        p.createSoftBodyAnchor(skinId, 21, self.boneId, 7+tmpIdx)
        p.createSoftBodyAnchor(skinId, 22, self.boneId, 7+tmpIdx)
        p.createSoftBodyAnchor(skinId, 23, self.boneId, 7+tmpIdx)

        p.createSoftBodyAnchor(skinId, 2, self.boneId, 8+tmpIdx)
        p.createSoftBodyAnchor(skinId, 3, self.boneId, 8+tmpIdx)
        p.createSoftBodyAnchor(skinId, 6, self.boneId, 8+tmpIdx)
        p.createSoftBodyAnchor(skinId, 7, self.boneId, 8+tmpIdx)

    def changeDynamics(self, anistropicFriction = [1,0.01,0.01], lateralFriction=2): # default anistropicFriction : move to X dir
        p.changeDynamics(self.boneId, -1, lateralFriction=lateralFriction, anisotropicFriction=anistropicFriction)

        for i in range(self.jointNum):
            p.changeDynamics(self.boneId, i, lateralFriction=lateralFriction, anisotropicFriction=anistropicFriction)

    def moveSin(self, steering=0.0, forces = 500, dt = 1./30., wavePeriod = 1.5, waveLength = 4, waveAmplitude = 0.8):
        segmentLength = self.gap + self.nodeLength
        scaleStart = 1.0
        waveFront = self.waveFront

        # start waves
        if (waveFront < segmentLength * 4.0):
            scaleStart = waveFront / (segmentLength * 4.0)

        targetPositions = np.full((self.jointNum, 4), None)
        for joint in range(self.jointNum - 1, -1, -1):
            segment = joint  # self.jointNum - 1 - joint

            # map segment to phase
            phase = (waveFront - (segment + 1) * segmentLength) / waveLength
            phase -= np.floor(phase)
            phase *= np.pi * 2.0

            # map phase to curvature
            pos = np.sin(phase) * scaleStart * waveAmplitude

            # steering = jointXYZActions[joint*4+1]
            if (steering > 0 and pos < 0):
                pos *= 1.0 / (1.0 + steering)

            if (steering < 0 and pos > 0):
                pos *= 1.0 / (1.0 - steering)

            pos += steering
            targetPositions[joint] = R.as_quat(R.from_euler('xyz', [0., pos, 0.]))

        p.setJointMotorControlMultiDofArray(self.boneId,
                                            self.jointIndices,
                                            p.POSITION_CONTROL,
                                            targetPositions,
                                            forces=forces)
        self.waveFront += dt / wavePeriod * waveLength
        p.stepSimulation()
        self.updateStates()

    def updateStates(self):
        self.headWorldPos, headWorldOriQuat = [np.array(t) for t in p.getBasePositionAndOrientation(self.boneId)]
        headWorldVel = np.array(p.getBaseVelocity(self.boneId)[0])

        nodeWorldStates = [[self.headWorldPos, headWorldVel]]

        for linkState in p.getLinkStates(self.boneId, self.linkIndices, computeLinkVelocity = 1):
            nodeWorldStates.append([np.array(linkState[0]), np.array(linkState[6])])

        nodeWorldStates = np.array(nodeWorldStates)

        self.tailWorldPos = nodeWorldStates[-1][0]
        self.midWorldPos, self.midWorldOri4M, self.midWorldVel = self.getMidWorldStates(nodeWorldStates)
        
        midWorldTrans4M = np.array(self.midWorldOri4M)
        midWorldTrans4M[3,:3] = self.midWorldPos
        self.midWorldTrans4M = midWorldTrans4M
        self.inv_midWorldTrans4M = np.linalg.inv(midWorldTrans4M)

        self.headLocalAheadDir = self.getHeadLocalAheadDir(self.headWorldPos, headWorldOriQuat, self.midWorldTrans4M)
        self.linksLocalPositionAndVelocity = self.getLinksLocalPositionAndVelocity(self.inv_midWorldTrans4M, nodeWorldStates)

    def getMidWorldStates(self, nodeWorldStates):
        pos, vel = 0, 1
        midIdx = int(self.nodeNum/2)

        # position
        if self.nodeNum % 2 == 1:
            midState = nodeWorldStates[midIdx]
            midPos =  np.array(midState[pos])
        else:
            midStates1, midStates2 = nodeWorldStates[midIdx:midIdx+2]
            midPos = (np.array(midStates1[pos]) + np.array(midStates2[pos])) / 2

        # orientation
        tailToHead = np.array(nodeWorldStates[0][pos]) - np.array(nodeWorldStates[self.nodeNum-1][pos])
        worldYaxis = np.array([0.,1.,0.])

        norm_tailToHead = np.linalg.norm(tailToHead)
        norm_worldYaxis = np.linalg.norm(worldYaxis)
        norm_headToY = np.linalg.norm(worldYaxis-tailToHead)
        cos = (norm_tailToHead**2 + norm_worldYaxis**2 - norm_headToY**2) / (2 * norm_tailToHead * norm_worldYaxis)
        theta = np.arccos(cos)
        rotAxis = np.cross(worldYaxis, tailToHead)
        rotAxis /= np.linalg.norm(rotAxis)
        rotVec = theta * rotAxis
        
        midOri = np.eye(4)
        midOri[:3,:3] = R.as_matrix(R.from_rotvec(rotVec))
        
        # velocity
        midVel = np.full((3,), 0.)
        for v in nodeWorldStates[:,vel]:
            midVel += np.array(v)

        return midPos, midOri, midVel

    def getHeadLocalAheadDir(self, headWorldPos, headWorldOriQuat, midWorldOri4M):
        if self.headXYZ == True:
            p.removeUserDebugItem(self.head_Z)

        # world coordinates
        mat = np.eye(4)
        mat[:3,:3] = np.reshape(p.getMatrixFromQuaternion(headWorldOriQuat),(3,3))
        
        _z = mat @ np.array([0.,0.,-1.,0.])
        _z /= np.linalg.norm(_z)

        self.head_Z = p.addUserDebugLine(headWorldPos, headWorldPos+_z[:3], lineColorRGB = [1,0,1])
        self.headXYZ = True

        localHeadDir = (midWorldOri4M.T @ _z)[:3]
        localHeadDir /= np.linalg.norm(localHeadDir)

        return localHeadDir

    def getLinksLocalPositionAndVelocity(self, inv_trans4M, nodeWorldStates):
        localPos = []
        localVel = []
        totalVel = np.array([0.,0.,0.])

        for wPos, wVel in nodeWorldStates:
            localPos.append((inv_trans4M @ np.concatenate((wPos, [1.])))[:3])

            lv = (inv_trans4M @ np.concatenate((wVel, [0.])))[:3]
            localVel.append(lv)
            totalVel += lv

        self.totalLocalVel = totalVel
        return np.concatenate((localPos, localVel)).reshape(3 * 2 * self.nodeNum,)
    
    def getLocalCoordinate(self, pos, isPos = 1.):
        return (self.inv_midWorldTrans4M @ np.concatenate((pos[:3], [isPos])))[:3]

    def getJointsTorqueSum(self):
        torque = p.getJointStatesMultiDof(self.boneId, self.jointIndices)
        torque_sum = np.sum([np.abs(i[3]) for i in torque])
        return torque_sum