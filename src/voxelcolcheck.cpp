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

VoxelCollisionChecker::VoxelCollisionChecker(EnvironmentBasePtr penv, VoxelGrid<int>& vg_in, Transform Tvg_in): OpenRAVE::CollisionCheckerBase(penv) /*vg_(1.0,1.0,1.0,0.5,Transform(),-10)*/ ,
    sdf_( 1.0,1.0,1.0, 0.5, Transform(), 1.0 )
{
    cout << __PRETTY_FUNCTION__ << " : voxel grid" << endl;

    // vg = vg_in;
    Tvg_ = Tvg_in;
    bDraw_ = true;
    bInitialized_ = true;
}

VoxelCollisionChecker::VoxelCollisionChecker(EnvironmentBasePtr penv): OpenRAVE::CollisionCheckerBase(penv), /*vg(1.0,1.0,1.0,0.5,Transform(),-10)*/
    sdf_( 1.0,1.0,1.0, 0.5, Transform(), 1.0 )
{
    cout << __PRETTY_FUNCTION__ << endl;

    bDraw_ = true;

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

        VoxelGrid<int> vg = createVoxelGrid( COMPUTE_NEW_VG, GetEnv(), robots[0], colbodies );
        sdf_ = createPDFfromVoxelGrid( vg, GetEnv(), graphptrs_ );
    }

    bInitialized_ = true;
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

    if(!bInitialized_)
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
    
    if(bDraw_)
    {
        vplotpoints_.resize(0);
        vplotcolors_.resize(0);
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
            float val = sdf_(sample.x,sample.y,sample.z).distance_square_;
            //RAVELOG_INFO("val: %f\n",val);
            if(val == 0)
            {
                report->numWithinTol++;
                vvPointsColliding[i][j] = true;
                if(bDraw_)
                {
                    vplotpoints_.push_back(sample);
                    vplotcolors_.push_back(1);
                    vplotcolors_.push_back(0);
                    vplotcolors_.push_back(0);
                }
            }
            else
            {
                vvPointsColliding[i][j] = false;
                if(bDraw_)
                {
                    vplotpoints_.push_back(sample);
                    vplotcolors_.push_back(0);
                    vplotcolors_.push_back(0);
                    vplotcolors_.push_back(1);
                }
            }
        }
    }
    
    if(bDraw_)
    {
        GraphHandlePtr figure = GetEnv()->plot3(&vplotpoints_[0].x,vplotpoints_.size(),sizeof(vplotpoints_[0]),20.0,&vplotcolors_[0],0);
        graphptrs_.push_back(figure);
    }
    
    if(report->numWithinTol)
        return true;
    else
        return false;
}
