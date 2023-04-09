import dartpy as dart

# create world
world = dart.simulation.World()
world.setGravity([0., 0., -9.81])

# add ground
urdfParser = dart.utils.DartLoader()
ground = urdfParser.parseSkeleton("dart://sample/urdf/KR5/ground.urdf")
world.addSkeleton(ground)

def viewer_func(world):
    node = dart.gui.osg.RealTimeWorldNode(world)
    viewer = dart.gui.osg.Viewer()
    viewer.addWorldNode(node)
    viewer.setUpViewInWindow(0, 0, 1200, 900)
    viewer.setCameraHomePosition([4., 4., 4.], [0., 0.25, 0.], [0., 0.8, 0.])
    viewer.run()

x = 1.1
cnt = 1
for j in range(3):
    for i in range(10):

        sphere = dart.dynamics.Skeleton(name="sphere")

   
        shape = dart.dynamics.SphereShape(radius=0.1)

        joint, body = sphere.createFreeJointAndBodyNodePair()

        body.setMass(mass=1)
      #  I = shape.computeInertia(mass=1)
      #  body.setMomentOfInertia(Ixx = I[0,0], Iyy=I[1,1], Izz=I[2,2],Ixy=I[0,1],Ixz=I[0,2],Iyz=I[1,2])

        shape_node = body.createShapeNode(shape)
        visual_aspect = shape_node.createVisualAspect()
        visual_aspect.setColor([1,0,0.5])
        
      #  collision_aspect = shape_node.createCollisionAspect()
      #  dynamics_aspect = shape_node.createDynamicsAspect()

        joint.setPosition(index=3,position=x)
        x -= 0.2
        world.addSkeleton(sphere)
        if i ==9:
            x = 1.1+(0.2*cnt)
            cnt += 1

    viewer_func(world)
