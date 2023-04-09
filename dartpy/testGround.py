import dartpy as dart

if __name__== "__main__":
    # create world
    world = dart.simulation.World()
    world.setGravity([0., 0., -9.81])

    # add ground
    urdfParser = dart.utils.DartLoader()
    ground = urdfParser.parseSkeleton("dart://sample/urdf/KR5/ground.urdf")
    world.addSkeleton(ground)

    node = dart.gui.osg.RealTimeWorldNode(world)
    viewer = dart.gui.osg.Viewer()
    viewer.addWorldNode(node)

    viewer.setUpViewInWindow(0, 0, 640, 480)
    viewer.setCameraHomePosition([0.8, 0.0, 0.8], [0, -0.25, 0], [0, 0.5, 0])
    viewer.run()
