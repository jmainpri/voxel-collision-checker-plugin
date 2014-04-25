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

#ifndef VOXELCOLCHECK_H
#define VOXELCOLCHECK_H

#include <iostream>

#include "collision_point.hpp"
#include "voxel_grid.hpp"
#include "propagation_distance_field.hpp"

//this collision checker currently fills the numWithinTol variable of the CollisionReport, nothing more
class VoxelCollisionChecker : public OpenRAVE::CollisionCheckerBase
{
public:
    VoxelCollisionChecker(EnvironmentBasePtr penv, distance_field::VoxelGrid<int>& vg_in, Transform Tvg_in);
    VoxelCollisionChecker(EnvironmentBasePtr penv);
    ~VoxelCollisionChecker() { graphptrs_.clear(); }

    /// Set basic collision options using the CollisionOptions enum
    virtual bool SetCollisionOptions(int collisionoptions) { std::cout << __PRETTY_FUNCTION__ << std::endl; return false;}
    virtual int GetCollisionOptions() const { std::cout << __PRETTY_FUNCTION__ << std::endl; return -1;}

    /// set and get various collision checker options
    /// \return true if command succeeded
    virtual bool SetCollisionOptions(std::ostream& sout, std::istream& sinput) { std::cout << __PRETTY_FUNCTION__ << std::endl; return true;}
    virtual void SetTolerance(dReal tolerance) {}

    /// called when environment sets this collision checker, checker assumes responsibility for KinBody::_pCollisionData
    /// checker should also gather all current bodies in the environment and put them in its collision space
    virtual bool InitEnvironment();

    /// called when environment switches to a different collision checker engine
    /// has to clear/deallocate any memory associated with KinBody::_pCollisionData
    virtual void DestroyEnvironment() {}

    /// notified when a new body has been initialized in the environment
    virtual bool InitKinBody(KinBodyPtr pbody);

    /// enables or disables a kinematic body from being considered in collisions
    /// \return true if operation succeeded
    virtual bool Enable(KinBodyConstPtr pbody, bool bEnable) { std::cout << __PRETTY_FUNCTION__ << std::endl; return true;}
    
    /// enables or disables a link from being considered in collisions
    /// \return true if operation succeeded
    virtual bool EnableLink(KinBody::LinkConstPtr pbody, bool bEnable) { std::cout << __PRETTY_FUNCTION__ << std::endl; return true;}

    virtual bool CheckCollision(KinBodyConstPtr pbody1, CollisionReportPtr report = CollisionReportPtr());

    virtual bool CheckCollision(KinBodyConstPtr pbody1, KinBodyConstPtr pbody2, CollisionReportPtr report = CollisionReportPtr()){ std::cout << __PRETTY_FUNCTION__ << std::endl; return false;}
    virtual bool CheckCollision(KinBody::LinkConstPtr plink, CollisionReportPtr report = CollisionReportPtr()){ std::cout << __PRETTY_FUNCTION__ << std::endl; return false;}
    virtual bool CheckCollision(KinBody::LinkConstPtr plink1, KinBody::LinkConstPtr plink2, CollisionReportPtr report = CollisionReportPtr()){ std::cout << __PRETTY_FUNCTION__ << std::endl; return false;}
    virtual bool CheckCollision(KinBody::LinkConstPtr plink, KinBodyConstPtr pbody, CollisionReportPtr report = CollisionReportPtr()){ std::cout << __PRETTY_FUNCTION__ << std::endl; return false;}
    
    virtual bool CheckCollision(KinBody::LinkConstPtr plink, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<KinBody::LinkConstPtr>& vlinkexcluded, CollisionReportPtr report = CollisionReportPtr()){ std::cout << __PRETTY_FUNCTION__ << std::endl; return false;}
    virtual bool CheckCollision(KinBodyConstPtr pbody, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<KinBody::LinkConstPtr>& vlinkexcluded, CollisionReportPtr report = CollisionReportPtr()){ std::cout << __PRETTY_FUNCTION__ << std::endl; return false;}
    
    virtual bool CheckCollision(const RAY& ray, KinBody::LinkConstPtr plink, CollisionReportPtr report = CollisionReportPtr()){ std::cout << __PRETTY_FUNCTION__ << std::endl; return false;}
    virtual bool CheckCollision(const RAY& ray, KinBodyConstPtr pbody, CollisionReportPtr report = CollisionReportPtr()){ std::cout << __PRETTY_FUNCTION__ << std::endl; return false;}
    virtual bool CheckCollision(const RAY& ray, CollisionReportPtr report = CollisionReportPtr()){std::cout << __PRETTY_FUNCTION__ << std::endl; return false;}

    /// checks self collision only with the links of the passed in body
    virtual bool CheckSelfCollision(KinBodyConstPtr pbody, CollisionReportPtr report = CollisionReportPtr()){ /*std::cout << __PRETTY_FUNCTION__ << std::endl; */ return false; }
    
    //voxel-specific check
    bool CheckCollision(KinBodyConstPtr pbody1, std::vector<std::vector<Vector> >& vvLinkPoints, CollisionReportPtr report = CollisionReportPtr());
    bool CheckCollision(KinBodyConstPtr pbody1, std::vector<std::vector<Vector> >& vvLinkPoints, std::vector<std::vector<bool> >& vvPointsColliding, CollisionReportPtr report = CollisionReportPtr());

    // Send commands to the collision checker
    virtual bool SendCommand( std::ostream& sout, std::istream& sinput );

    bool bInitialized_;


protected:

    bool InitModule();
    bool SetDimension( std::istream& sinput );
    bool Initialize( std::istream& sinput );
    bool SetCollisionPointsRadii( std::istream& sinput );
    void RedrawCollisionPoints();
    void CreateCollisionPoints( RobotBasePtr robot );
    bool GetCollisionPointPotentialGradient( distance_field::CollisionPoint& coll_point, const OpenRAVE::Vector& collision_point_pos, double& field_distance, double& potential, OpenRAVE::Vector& gradient) ;

    std::vector< distance_field::CollisionPoint > collision_points_;

    distance_field::PropagationDistanceField sdf_;

    Transform Tvg_; //transform of the voxel grid center
    
    bool bDraw_;
    std::vector<  boost::shared_ptr<void>  > graphptrs_;
    std::vector<RaveVector<float> > vplotpoints_;
    std::vector<float> vplotcolors_;

    std::vector<OpenRAVE::KinBodyPtr> colbodies_;
    OpenRAVE::Vector dimension_;
    OpenRAVE::Vector offset_;
    float voxel_size_;
    bool robot_centered_;
    std::vector<float> radii_;
    double draw_distance_;
    double draw_color_threshold_;
};

#endif
