#!/usr/bin/env python
# Copyright (c) 2014 Worcester Polytcchnic Institute
#   Author: Jim Mainprice <jmainprice@wpi.edu>
#
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Intel Corporation nor Carnegie Mellon University,
#       nor the names of their contributors, may be used to endorse or
#       promote products derived from this software without specific prior
#       written permission.
#
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#   ARE DISCLAIMED. IN NO EVENT SHALL INTEL CORPORATION OR CARNEGIE MELLON
#   UNIVERSITY BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
#   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
#   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
#   OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
#   WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
#   OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
#   ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# -*- coding: utf-8 -*-

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
    orEnv.Load('robots/pr2-beta-static.zae')
    orEnv.Load('data/shelf.kinbody.xml')

    T_cam = array(  [[ 0.81240217, -0.20717684,  0.54505089, -1.11184239], \
                     [-0.58198477, -0.34581952,  0.73600448, -2.30210519], \
                     [ 0.03600615, -0.91514295, -0.40151829,  2.17947888], \
                     [ 0.,          0.,          0.,          1.        ]] )

    orEnv.GetViewer().SetCamera( T_cam )

    T = eye(4)
    T[0,3] = 0.0
    T[1,3] = 0.0
    T[2,3] = 0.0

    for b in orEnv.GetBodies() :
        print "name : ", b.GetName()
        if  b.GetName() == "Shelf":
            shelf = b

    # Get body
    shelf.SetTransform( array( MakeTransform( rodrigues([-pi/2,0,0]), matrix([0.7,-0.5,0]) ) ) )

    # Get robot
    robot = orEnv.GetRobots()[0]
    robot.SetTransform( T )

    # Set active manipulator
    robot.SetActiveManipulator(1)
    # indices = robot.GetActiveManipulator().GetArmIndices()
    indices = [27, 28, 29, 30, 31, 32, 33]

    # Print the limits
    for jIdx, j in enumerate(robot.GetJoints()):
        if jIdx not in indices :
            continue
        print "%s, \t%.3f, \t%.3f" % ( j.GetName() , j.GetLimits()[0] , j.GetLimits()[1] )

    robot.SetActiveDOFs( indices )
    robot.SetDOFValues( [-0.2,0.5,0.5,-0.6,-1.5,-1,0], indices )

    # Init collision checker
    collisionChecker = RaveCreateCollisionChecker( orEnv,'VoxelColChecker')
    collisionChecker.SendCommand('SetDimension robotcentered extent 1.5 2.0 1.5 offset 0 -1 0.5')
    collisionChecker.SendCommand('Initialize')

    orEnv.SetCollisionChecker( collisionChecker )

    print "checking collision for robot : " + robot.GetName()
    report = CollisionReport()
    in_collision = orEnv.CheckCollision( robot, report )
    print 'in_collision: ', in_collision
    print 'mindist: ', report.minDistance
    # print 'contacts: ', report.contacts
    for c in report.contacts :
        print "collision point potential : " , str( c.depth )
    
    print "Press return to exit."
    sys.stdin.readline()

    print orEnv.GetViewer().GetCameraTransform()
