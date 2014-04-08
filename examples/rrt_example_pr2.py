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

# openrave-robot.py /your/path/to/your.robot.xml --info=joints
# On that page you can find more examples on how to use openrave-robot.py.
# http://openrave.org/docs/latest_stable/command_line_tools/

from openravepy import *
from numpy import *
from misc_transform import *

import time
import sys

if __name__ == "__main__":
    print "START OPENRAVE"
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

    # Set the initial configuration
    robot.SetActiveManipulator(1)
    indices = robot.GetActiveManipulator().GetArmIndices()
    print indices
    # Print the limits
    for jIdx, j in enumerate(robot.GetJoints()):
        if jIdx not in indices :
            continue
        print "%s, \t%.3f, \t%.3f" % ( j.GetName() , j.GetLimits()[0] , j.GetLimits()[1] )

    robot.SetActiveDOFs( indices )
    robot.SetDOFValues( [-1,0.5,0.5,-0.6,-1.5], indices )

    print robot.GetActiveDOFIndices()

    planner = RaveCreatePlanner( orEnv, 'birrt' )
    params = Planner.PlannerParameters()
    params.SetRobotActiveJoints( robot )

    # Set the goal configuration
    params.SetGoalConfig( [0.3,0.5,0.5,-0.6,-1.5] )
    planner.InitPlan( robot, params )

    traj = RaveCreateTrajectory( orEnv, '' )

    # Number of configurations drawing the path
    nb_of_config = 10

    if planner.PlanPath(traj) :
        
        # Write to file
        f = open( "trajectory.txt", 'w')
        f.write( traj.serialize() )
        f.close()

        # Draw trace
        for t in linspace( 0, traj.GetDuration(), nb_of_config ):
            newrobot = RaveCreateRobot( orEnv, robot.GetXMLId() )
            newrobot.Clone( robot, 0 )
            for link in newrobot.GetLinks():
               for geom in link.GetGeometries():
                   geom.SetTransparency( 0.3 )
            orEnv.Add( newrobot, True )
            q = traj.Sample( t ) # get configuration
            print q
            newrobot.SetDOFValues( q[0:5] , indices )
    
    print "Press return to exit."
    sys.stdin.readline()
