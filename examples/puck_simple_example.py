#!/usr/bin/env python
# Copyright (c) 2014 Worcester Polytcchnic Institute
#   Author: Jim Mainprice <jmainprice@wpi.edu>

from openravepy import *
from numpy import *
from misc_transform import *

import time
import sys

if __name__ == "__main__":
      
    # load the environment if it is not already loaded
    try:
        orEnv
    except NameError:
        orEnv = Environment()
        orEnv.SetViewer('qtcoin')
    
    # load the robot into the environment
    orEnv.Reset()
    orEnv.Load( '../models/stones.env.xml' )

    T_cam = ([[  2.98714915e-01,   9.44717010e-01,  -1.35200486e-01,   2.94798950e+02], \
              [  9.53920362e-01,  -2.99784043e-01,   1.28635264e-02,   3.90384369e+02], \
              [ -2.83785562e-02,  -1.32813024e-01,  -9.90734757e-01,   7.22618408e+02], \
              [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,  1.00000000e+00]])

    orEnv.GetViewer().SetCamera( T_cam )

    # Get robot
    robot = orEnv.GetRobots()[0]    

    # Set position
    indices = robot.GetActiveManipulator().GetArmIndices()
    robot.SetDOFValues( [200,340], indices )

    # Disable Box0 link
    robot.GetLink('Box0').Enable( False )
 
    # Init collision checker
    collisionChecker = RaveCreateCollisionChecker( orEnv,'VoxelColChecker')
    collisionChecker.SendCommand('SetDimension extent 500 800 30 voxelsize 5')
    collisionChecker.SendCommand('Initialize')
    collisionChecker.SendCommand('SetDrawing on')
    orEnv.SetCollisionChecker( collisionChecker )

    print "checking collision for robot : " + robot.GetName()
    report = CollisionReport()
    in_collision = orEnv.CheckCollision( robot, report )
    print 'in_collision: ', in_collision
    print 'mindist: ', report.minDistance

    for c in report.contacts :
        print "collision point potential : " , str( c.depth )
    
    print "Press return to exit."
    sys.stdin.readline()

    print orEnv.GetViewer().GetCameraTransform()
