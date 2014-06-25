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

/** \author Dmitry Berenson and Jim Mainprice */

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

    InitModule();
}

VoxelCollisionChecker::VoxelCollisionChecker( EnvironmentBasePtr penv ): OpenRAVE::CollisionCheckerBase(penv), /*vg(1.0,1.0,1.0,0.5,Transform(),-10)*/
    sdf_( 1.0,1.0,1.0, 0.5, Transform(), 1.0 )
{
    cout << __PRETTY_FUNCTION__ << endl;

    InitModule();
}

bool VoxelCollisionChecker::InitModule()
{

    bDraw_ = true;
    draw_distance_ = 0.15;
    draw_color_threshold_ = 0.40;
    bInitialized_ = false;
    dimension_ = OpenRAVE::Vector(1,1,1);
    offset_ = OpenRAVE::Vector(0,0,0);
    voxel_size_ = 0.05;
    collision_points_.clear();
    robot_centered_ = false;
    radii_.clear();
    radii_.push_back( std::make_pair( true, 0.20 ) );

    RegisterFunctions();

    return true;
}

void VoxelCollisionChecker::RegisterFunctions()
{
    RegisterCommand("setdimension",boost::bind(&VoxelCollisionChecker::SetDimension,this,_2),"returns true if ok");
    RegisterCommand("initialize",boost::bind(&VoxelCollisionChecker::Initialize,this,_2),"returns true if ok");
    RegisterCommand("setcollisionpointsradii",boost::bind(&VoxelCollisionChecker::SetCollisionPointsRadii,this,_2),"returns true if ok");
    RegisterCommand("setdrawing",boost::bind(&VoxelCollisionChecker::SetDrawing,this,_2),"returns true if ok");
}

void VoxelCollisionChecker::Clone(InterfaceBaseConstPtr preference, int cloningoptions)
{
    const VoxelCollisionChecker* vox_checker = dynamic_cast< const VoxelCollisionChecker* >(preference.get());

    bInitialized_ = vox_checker->bInitialized_;
    collision_points_ = vox_checker->collision_points_;
    sdf_ = vox_checker->sdf_;
    Tvg_ = vox_checker->Tvg_; //transform of the voxel grid center
    bDraw_ = vox_checker->bDraw_;
    graphptrs_ = vox_checker->graphptrs_;
    vplotpoints_ = vox_checker->vplotpoints_;
    vplotcolors_ = vox_checker->vplotcolors_;
    dimension_ = vox_checker->dimension_;
    offset_ = vox_checker->offset_;
    voxel_size_= vox_checker->voxel_size_ ;
    robot_centered_ = vox_checker->robot_centered_;
    radii_ = vox_checker->radii_;
    draw_distance_ = vox_checker->draw_distance_;
    draw_color_threshold_ = vox_checker->draw_color_threshold_;

    colbodies_.clear();
    GetEnv()->GetBodies( colbodies_ );
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
bool VoxelCollisionChecker::InitKinBody( KinBodyPtr pbody )
{
//    std::cout << __PRETTY_FUNCTION__ << std::endl;
    UserDataPtr pdata( new UserData() );
    SetCollisionData( pbody, pdata );
    return true;
}

bool VoxelCollisionChecker::GetCollisionPointPotentialGradient( const distance_field::CollisionPoint& collision_point, const OpenRAVE::Vector& p, double& field_distance, double& potential, OpenRAVE::Vector& pg ) const
{
    if( !bInitialized_ ) {
        RAVELOG_INFO("ERROR: not initialized!\n");
        return false;
    }

    // Function by Mrinal Kalakrishnan

    OpenRAVE::Vector field_gradient;

    // Compute the distance gradient and distance to nearest obstacle
    double px = pg.x;
    double py = pg.y;
    double pz = pg.z;
    field_distance = sdf_.getDistanceGradient( p.x, p.y, p.z, px, py, pz );

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

// **********************************************
// MAIN function for robot collision checking
// **********************************************
bool VoxelCollisionChecker::CheckCollision( KinBodyConstPtr pbody1, CollisionReportPtr report )
{
//    cout << __PRETTY_FUNCTION__ << std::endl;
//    cout << " -- main collision detection function" << endl;

    if(!bInitialized_)
    {
        RAVELOG_INFO("VoxelCollisionChecker ERROR: VoxelCollisionChecker is not initialized!\n");
        return false;
    }

    bool do_report = ( report.get() != NULL );
    bool in_collision( false );
    bool is_point_in_collision( false );
    OpenRAVE::Vector p, pg;
    double distance_obst, potential;

    if( do_report ){
        report->contacts.resize( collision_points_.size() );
        report->minDistance = std::numeric_limits<dReal>::max();
    }

    if( !collision_points_.empty() )
    {
        OpenRAVE::KinBody::JointPtr joint = pbody1->GetJoint( collision_points_[0].getJointName() );

        for( size_t i =0; i<collision_points_.size() ; i ++ )
        {
            if( joint->GetName() != collision_points_[i].getJointName() )
                joint = pbody1->GetJoint( collision_points_[i].getJointName() );

            collision_points_[i].m_is_colliding = false;
            collision_points_[i].getTransformMatrixedPosition( joint->GetHierarchyChildLink()->GetTransform(), p );

            if( do_report ) // Complete report case (compute potential)
            {
                is_point_in_collision = GetCollisionPointPotentialGradient( collision_points_[i], p, distance_obst, potential, pg );

                double distance = ( distance_obst - collision_points_[i].getRadius() );
                if( distance < report->minDistance )
                    report->minDistance = distance;

                report->contacts[i].depth = potential; // depth should be "distance" (highjack for optimizers)
                report->contacts[i].pos = p;
            }
            else // Simple case check distance
            {
                distance_obst = sdf_.getDistance( sdf_( p.x, p.y, p.z) );
                is_point_in_collision = ( distance_obst <= collision_points_[i].getRadius() );
            }

            if( is_point_in_collision  )
            {
                collision_points_[i].m_is_colliding = true;
                in_collision = true;
            }
        }
    }

//    std::map<int,bool> body_colliding;

//    for( size_t i=0;i<collision_points_.size();i++)
//    {
//        int body_id = collision_points_[i].getSegmentNumber();
//        if( body_colliding[body_id] )
//            continue;

//        if( collision_points_[i].m_is_colliding ) {
//            body_colliding[body_id] = true;
//            cout << "body_id : " << body_id << ", " << collision_points_[i].getJointName() << " is colliding" << endl;
//        }
//    }

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

// -------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------

bool VoxelCollisionChecker::SendCommand( std::ostream& sout, std::istream& sinput )
{
    OpenRAVE::CollisionCheckerBase::SendCommand( sout, sinput );
    return true;
}

void VoxelCollisionChecker::CreateCollisionPoints( RobotBasePtr robot )
{
    collision_points_.clear();

    drawClearHandles();

    if( robot->GetName() == "Puck" ) // Special case for the puck robot
    {
        cout << "Compute collision points for " << robot->GetName() << endl;
        radii_.clear();
        radii_.push_back( std::make_pair( true, 20.0 ) );
        collision_points_ = createCollionPointsForRobot( GetEnv(), robot, radii_, true );
        setDrawingDistance( 15, 40 );
    }
    else // General case
    {
        cout << "Compute collision points for " << robot->GetName() << endl;
        collision_points_ = createCollionPointsForRobot( GetEnv(), robot, radii_ );
        setDrawingDistance( draw_distance_, draw_color_threshold_ );
    }

    cout << "collision_points_.size() : " << collision_points_.size() << endl;

    if( bDraw_ )
    {
        if( bInitialized_ )
            drawPDF( sdf_, GetEnv() ); // Draws PDF here

        RedrawCollisionPoints();
    }
}

void VoxelCollisionChecker::RedrawCollisionPoints()
{
    graphptrs_.clear();

    std::vector<RobotBasePtr> robots;
    GetEnv()->GetRobots( robots );

    for( size_t i =0; i < collision_points_.size() ; i ++ )
    {
        int segment = collision_points_[i].getSegmentNumber();
//        cout << "collision_points_[i].getSegmentNumber() : " << segment << " , " << radii_[ segment ].first << endl;
        if( segment < int(radii_.size()) && radii_[ segment ].first )
        {
            collision_points_[i].draw( graphptrs_, robots[0], GetEnv() );
        }
    }
}

bool VoxelCollisionChecker::SetDrawing( std::istream& sinput )
{
    std::string cmd;
    while(!sinput.eof())
    {
        sinput >> cmd;
        if( !sinput )
            break;

        if( cmd == "off" )
        {
            graphptrs_.clear();
            drawClearHandles();
            bDraw_ = false;
        }
        if( cmd == "on" )
        {
            bDraw_ = true;
            if( bInitialized_ )
                drawPDF( sdf_, GetEnv() ); // Draws PDF here
        }
        else break;
        if( !sinput ) {
            RAVELOG_DEBUG("failed\n");
            return false;
        }
    }

    return true;
}

bool VoxelCollisionChecker::SetCollisionPointsRadii( std::istream& sinput )
{
    radii_.clear();

    std::string cmd;
    while(!sinput.eof())
    {
        sinput >> cmd;
        if( !sinput )
            break;

        if( cmd == "radii" )
        {
            cout << cmd << endl;
            // note that this appends to goals, does not overwrite them
            size_t temp;
            sinput >> temp;
            radii_.resize( temp );
            for( size_t i=0; i<temp; i++ ) {
                radii_[i].first = true;
                sinput >> radii_[i].second;
            }
        }
        else if( cmd == "activation" )
        {
            cout << cmd << endl;
            // note that this appends to goals, does not overwrite them
            size_t temp;
            sinput >> temp;
            radii_.resize( temp );
            for( size_t i=0; i<temp; i++ ) {
                sinput >> radii_[i].first;
                cout << "radii_[" << i << "] : " << radii_[i].first << endl;
            }
        }
        else break;
        if( !sinput ) {
            RAVELOG_DEBUG("failed\n");
            return false;
        }
    }

    std::vector<RobotBasePtr> robots;
    GetEnv()->GetRobots( robots );
    CreateCollisionPoints( robots[0] );

    return true;
}

bool VoxelCollisionChecker::SetDimension( std::istream& sinput )
{
    std::string cmd;

    while(!sinput.eof())
    {
        sinput >> cmd;
        if( !sinput )
            break;

        if( cmd == "voxelsize" )
        {
            sinput >> voxel_size_;
        }
        else if( cmd == "extent" )
        {
            for(size_t i = 0; i < 3; i++)
                sinput >> dimension_[i];
        }
        else if( cmd == "offset" )
        {
            cout << cmd << endl;
            for(size_t i = 0; i < 3; i++)
                sinput >> offset_[i];
            cout << offset_ << endl;
        }
        else if( cmd == "robotcentered" )
        {
            robot_centered_ = true;
        }
        else break;
        if( !sinput ) {
            RAVELOG_DEBUG("failed\n");
            return false;
        }
    }

    return true;
}

bool VoxelCollisionChecker::Initialize( std::istream& sinput )
{
    std::vector<RobotBasePtr> robots;
    GetEnv()->GetRobots( robots );

    if( !robots.empty() )
    {
        // Get list of obstacles bodies
        colbodies_.clear();
        GetEnv()->GetBodies( colbodies_ );

        bool no_robot = true;
        if( no_robot )
        {
            int robot_id = 0;
            for( size_t i=0; i<colbodies_.size(); i++ )
            {
                if( colbodies_[i]->GetName() == robots[0]->GetName() )
                    robot_id = i;
            }

            // Remove robot from collision body list
            colbodies_.erase( colbodies_.begin() + robot_id );
        }

        // Get robot transform and apply offset

        OpenRAVE::Transform origin;

        if( robot_centered_ )
            origin = robots[0]->GetTransform();

        origin.trans += offset_;

        // Set voxel grid dimension and origin
        setVoxelGridSize( dimension_.x, dimension_.y, dimension_.z, voxel_size_, origin );

        cout << "create voxel grid" << endl;

        // Create Signed Distance Feild
        VoxelGrid<int> vg = createVoxelGrid( COMPUTE_NEW_VG, GetEnv(), robots[0], colbodies_ );
        sdf_ = createPDFfromVoxelGrid( vg, GetEnv() );

        // Create collision points
        CreateCollisionPoints( robots[0] );

        // Initialization done!!!
        bInitialized_ = true;
        return true;
    }

    return false;
}
