// Copyright (C) 2006-2008 Carnegie Mellon University (dberenso@cs.cmu.edu)
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "stdafx.h"

VoxelCollisionChecker::VoxelCollisionChecker(EnvironmentBasePtr penv, VoxelGrid<int>& vg_in, Transform Tvg_in): OpenRAVE::CollisionCheckerBase(penv), vg(1.0,1.0,1.0,0.5,Transform(),-10)
{
    cout << __PRETTY_FUNCTION__ << " : voxel grid" << endl;

    vg = vg_in;
    Tvg = Tvg_in;
    bDraw = true;
    bInitialized = true;
}

VoxelCollisionChecker::VoxelCollisionChecker(EnvironmentBasePtr penv): OpenRAVE::CollisionCheckerBase(penv), vg(1.0,1.0,1.0,0.5,Transform(),-10)
{
    cout << __PRETTY_FUNCTION__ << endl;

    bDraw = true;

    std::vector<RobotBasePtr> robots;
    penv->GetRobots( robots );

    if( !robots.empty() ) {

        std::vector<KinBodyPtr> colbodies;
        penv->GetBodies( colbodies );

        int robot_id = 0;

        for( size_t i = 0; i < colbodies.size(); i++ )
        {
            if( colbodies[i]->GetName() == robots[0]->GetName() )
            {
                robot_id = i;
            }
        }

        colbodies.erase( colbodies.begin() + robot_id );

        vg = createVoxelGrid( COMPUTE_NEW_VG, GetEnv(), robots[0], colbodies );
    }

    bInitialized = false;
}

bool VoxelCollisionChecker::CheckCollision(KinBodyConstPtr pbody1, std::vector<std::vector<Vector> >& vvLinkPoints, CollisionReportPtr report)
{
    cout << __PRETTY_FUNCTION__ << endl;

    std::vector<std::vector<bool> > vvPointsColliding;
    return CheckCollision( pbody1, vvLinkPoints, vvPointsColliding, report );
}

bool VoxelCollisionChecker::CheckCollision(KinBodyConstPtr pbody1, std::vector<std::vector<Vector> >& vvLinkPoints, std::vector<std::vector<bool> >& vvPointsColliding, CollisionReportPtr report)
{
    cout << __PRETTY_FUNCTION__ << endl;

    if(!bInitialized)
    {
        RAVELOG_INFO("VoxelCollisionChecker ERROR: VoxelCollisionChecker is not initialized!\n");
        return false;
    }
    
    if(pbody1->GetLinks().size() != vvLinkPoints.size())
    {
        RAVELOG_INFO("VoxelCollisionChecker ERROR: Number of links in body (%d) and number of links in vvLinkPoints (%d) are different!\n",pbody1->GetLinks().size(),vvLinkPoints.size());
        return false;
    }
    
    if( !report )
        report.reset(new CollisionReport());
    
    if(bDraw)
    {
        vplotpoints.resize(0);
        vplotcolors.resize(0);
    }

    vvPointsColliding.resize(pbody1->GetLinks().size());
    
    report->numWithinTol = 0;
    

    for( size_t i = 0; i < pbody1->GetLinks().size(); i++)
    {
        Transform temptm = pbody1->GetLinks()[i]->GetTransform();
        vvPointsColliding[i].resize(vvLinkPoints[i].size());

        for( size_t j = 0; j < vvLinkPoints[i].size(); j++)
        {
            //RAVELOG_INFO("linktrans: %f %f %f\n",temptm.trans.x,temptm.trans.y,temptm.trans.z);
            Vector sample = temptm*vvLinkPoints[i][j];
            float val = vg(sample.x,sample.y,sample.z);
            //RAVELOG_INFO("val: %f\n",val);
            if(val > 0)
            {
                report->numWithinTol++;
                vvPointsColliding[i][j] = true;
                if(bDraw)
                {
                    vplotpoints.push_back(sample);
                    vplotcolors.push_back(1);
                    vplotcolors.push_back(0);
                    vplotcolors.push_back(0);
                }
            }
            else
            {
                vvPointsColliding[i][j] = false;
                if(bDraw)
                {
                    vplotpoints.push_back(sample);
                    vplotcolors.push_back(0);
                    vplotcolors.push_back(0);
                    vplotcolors.push_back(1);
                }
            }
        }
    }
    
    if(bDraw)
    {
        GraphHandlePtr figure = GetEnv()->plot3(&vplotpoints[0].x,vplotpoints.size(),sizeof(vplotpoints[0]),20.0,&vplotcolors[0],0);
        graphptrs.push_back(figure);
    }
    
    if(report->numWithinTol)
        return true;
    else
        return false;
    
}
