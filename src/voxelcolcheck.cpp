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

#include "voxelcolcheck.hpp"
#include "voxel_grid.hpp"
#include "propagation_distance_field.hpp"
#include "collision_point.hpp"
#include "init_obstacle_field.hpp"

using namespace distance_field;

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

    cout << __PRETTY_FUNCTION__ << endl;

    std::vector<RobotBasePtr> robots;
    GetEnv()->GetRobots( robots );

    if( !robots.empty() )
    {
        // List obstacle bodies
        std::vector<KinBodyPtr> colbodies;
        GetEnv()->GetBodies( colbodies );

        int robot_id = 0;

        for( size_t i = 0; i < colbodies.size(); i++ )
        {
            if( colbodies[i]->GetName() == robots[0]->GetName() )
            {
                robot_id = i;
            }
        }

        colbodies.erase( colbodies.begin() + robot_id );

        if( robots[0]->GetName() == "Puck" )
        {
            setVoxelGridSize( 500, 800, 30, 5 );
            setDrawingDistance( 15, 50 );
        }

        if( robots[0]->GetName() == "pr2" )
        {
            Transform origin = robots[0]->GetTransform();
            origin.trans.y -= 1.00;
            origin.trans.z += 0.50;
            setVoxelGridSize( 2.0, 2.0, 1.5, 0.025, origin );
            setDrawingDistance( .10, .30 );
        }

        // Create sdf
        VoxelGrid<int> vg = createVoxelGrid( COMPUTE_NEW_VG, GetEnv(), robots[0], colbodies );
        sdf_ = createPDFfromVoxelGrid( vg, GetEnv() );

        CreateCollisionPoints( robots[0] );

        bInitialized_ = true;
    }
}

bool VoxelCollisionChecker::InitEnvironment()
{
    // Gegister bodies
    std::vector<KinBodyPtr> allbodies;
    GetEnv()->GetBodies( allbodies );

    for( size_t i = 0; i < allbodies.size(); i++ )
    {
        cout << "name : " << allbodies[i]->GetName() << endl;
        InitKinBody( allbodies[i] );
    }

    return true;
}

/// notified when a new body has been initialized in the environment
bool VoxelCollisionChecker::InitKinBody(KinBodyPtr pbody)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    UserDataPtr pdata( new UserData() );
    SetCollisionData( pbody, pdata );
    return true;
}

void VoxelCollisionChecker::CreateCollisionPoints( RobotBasePtr robot )
{
    collision_points_.clear();

    if( robot->GetName() == "pr2" )
    {
            cout << "Compute collision points for " << robot->GetName() << endl;
            collision_points_ = createCollionPointsForPr2( robot );
    }
    else if( robot->GetName() == "Puck" )
    {
            cout << "Compute collision points for " << robot->GetName() << endl;
            collision_points_ = createCollionPointsForPuck( robot );
    }
    else {
        RAVELOG_INFO("Does not know how to compute collision points for kinbody : %s\n" , robot->GetName().c_str() );
    }

    RedrawCollisionPoints();
}

void VoxelCollisionChecker::RedrawCollisionPoints()
{
    graphptrs_.clear();

    for( size_t i =0; i < collision_points_.size() ; i ++ )
    {
        collision_points_[i].draw( graphptrs_, GetEnv() );
    }
}

// main function for collision checking of robot
bool VoxelCollisionChecker::CheckCollision( KinBodyConstPtr pbody1, CollisionReportPtr report )
{
//    cout << __PRETTY_FUNCTION__ << std::endl;
//    cout << " -- main collision detection function" << endl;

    if( report.get() != NULL ){
        report->contacts.resize( collision_points_.size() );
        report->minDistance = std::numeric_limits<dReal>::max();
    }

    bool in_collision( false );
    bool is_point_in_collision( false );

    OpenRAVE::Vector p, pg;
    double distance_obst, potential;

    for( size_t i =0; i<collision_points_.size() ; i ++ )
    {
        collision_points_[i].m_is_colliding = false;
        collision_points_[i].getTransformMatrixedPosition( p );

        is_point_in_collision = GetCollisionPointPotentialGradient( collision_points_[i], p, distance_obst, potential, pg );
        // distance_obst = sdf_.getDistance( sdf_( p.x, p.y, p.z) );

        if( is_point_in_collision  )
        {
            collision_points_[i].m_is_colliding = true;
            in_collision = true;
        }

        if( report.get() != NULL )
        {
            if( distance_obst < report->minDistance )
            {
                report->minDistance = distance_obst;
            }
            report->contacts[i].depth = potential;
            report->contacts[i].pos = p;
        }

//        cout << "joint name : " << collision_points_[i].getJointName() ;
//        cout << std::setprecision(2) << " , \t potential : " << potential ;
//        cout << " , \t in collision : " << is_point_in_collision ;
//        cout << endl;

        // report->contacts[i].depth = distance_obst;

        // cout << " distance_obst : " << distance_obst << endl;

        // OpenRAVE::RaveVector<float> vcolors(1.0, collision_points_[i].m_is_colliding ? 0.0 : 1.0 ,0.0,0.1);
        // std::vector<OpenRAVE::RaveVector<float> > vpoints;
        // vpoints.push_back( p );
        // graphptrs_.push_back( GetEnv()->plot3( &vpoints[0].x, vpoints.size(), sizeof( vpoints[0]), collision_points_[i].getRadius(), vcolors, 1 ) );
    }

    // RedrawCollisionPoints();

    // cout << " report->minDistance : " << report->minDistance << endl;
    // cout << " in_collision : " << in_collision << endl;

    return in_collision;
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

bool VoxelCollisionChecker::GetCollisionPointPotentialGradient( distance_field::CollisionPoint& collision_point, const OpenRAVE::Vector& p, double& field_distance, double& potential, OpenRAVE::Vector& pg ) const
{
    OpenRAVE::Vector field_gradient;

    // Compute the distance gradient and distance to nearest obstacle
    field_distance = sdf_.getDistanceGradient( p.x, p.y, p.z, pg.x, pg.y, pg.z );

    double d = field_distance - collision_point.getRadius();

    // three cases below
    if (d >= collision_point.getClearance())
    {
        potential = 0.0;
        pg = OpenRAVE::Vector(0,0,0);
    }
    else if (d >= 0.0)
    {
        double diff = ( d - collision_point.getClearance() );
        double gradient_magnitude = diff * collision_point.getInvClearance(); // (diff / clearance)
        potential = 0.5 * gradient_magnitude * diff;
        pg = gradient_magnitude * field_gradient;
    }
    else // if d < 0.0
    {
        pg = field_gradient;
        potential = -d + 0.5 * collision_point.getClearance();
    }

    return (field_distance <= collision_point.getRadius()); // true if point is in collision
}
