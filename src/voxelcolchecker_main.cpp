/* Copyright (c) 2010 Carnegie Mellon University and Intel Corporation
   Author: Jim Mainprice <jmainprice@wpi.edu>

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
     * Neither the name of Intel Corporation nor Carnegie Mellon University,
       nor the names of their contributors, may be used to endorse or
       promote products derived from this software without specific prior
       written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL INTEL CORPORATION OR CARNEGIE MELLON
   UNIVERSITY BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
   OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
   WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
   OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
   ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/** \file voxelcolchecker_main.cpp
    \brief Registers generalik iksolver with openrave.
 */

/*! \mainpage Voxel Collision Checker Openrave Plugin
 *
 *
 * The generalik plugin is an inverse kinematics solver which implements an iterative Jacobian pseudo-inverse
 * algorithm. The solver can also compensate for balance constraints in the null space of the Jacobian pseudo-inverse.
 * No specific manipulator needs to be active, all active joints of the robot are used. This IK solver will only return
 * one answer and may get stuck in joint limits or singularities, however it works with any number of active joints. If
 * the solver cannot find a solution, it will return the closest configuration it could find.
 */

#include "stdafx.h"
#include <rave/plugin.h>

InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_CollisionChecker && interfacename == "voxelcolchecker" ) {
        return InterfaceBasePtr(new VoxelCollisionChecker(penv));
    }
    return InterfaceBasePtr();
}

void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[PT_CollisionChecker].push_back("VoxelColChecker");
}

RAVE_PLUGIN_API void DestroyPlugin()
{
    RAVELOG_INFO("destroying plugin\n");
}
