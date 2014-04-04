/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \author Mrinal Kalakrishnan */
/** modified Lillian Chang to remove dependencies on ros and bullet */

#ifndef DF_DISTANCE_FIELD_H_
#define DF_DISTANCE_FIELD_H_

#include <voxel_grid.hpp>
//#include <LinearMath/btVector3.h>
#include <vector>
#include <list>
//#include <ros/ros.h>
//#include <visualization_msgs/Marker.h>
//#include <mapping_msgs/CollisionMap.h>

class point3d 
{
    
public:
    double x,y,z;
    
    /* constructors */
    point3d() {
        this->x = 0.;
        this->y = 0.;
        this->z = 0.;
    }
    point3d( double x, double y, double z) {
        this->x = x;
        this->y = y;
        this->z = z;
    }
};


namespace distance_field
{

/**
 * \brief A VoxelGrid that can convert a set of obstacle points into a distance field.
 *
 * It computes the distance transform of the input points, and stores the distance to
 * the closest obstacle in each voxel. Also available is the location of the closest point,
 * and the gradient of the field at a point. Expansion of obstacles is performed upto a given
 * radius.
 *
 * This is an abstract base class, current implementations include PropagationDistanceField
 * and PFDistanceField.
 */
template <typename T>
class DistanceField: public VoxelGrid<T>
{
public:
    /**
   * \brief Constructor for the VoxelGrid.
   *
   * @param size_x Size (x axis) of the container in meters
   * @param size_y Size (y axis) of the container in meters
   * @param size_z Size (z axis) of the container in meters
   * @param resolution: resolution (size of a single cell) in meters
   * @param origin_in: transform of origin of the container
   * @param default_object The object to return for an out-of-bounds query
   */
    DistanceField(double size_x, double size_y, double size_z, double resolution,
                  Transform origin, T default_object);

    virtual ~DistanceField();


    /**
   * \brief Add (and expand) a set of points to the distance field.
   *
   * This function will incrementally add the given points and update the distance field
   * correspondingly. Use the reset() function if you need to remove all points and start
   * afresh.
   */
    virtual void addPointsToField(const std::vector<point3d> points)=0;

    /**
   * \brief Adds the points in a collision map to the distance field.
   */
    //void addCollisionMapToField(const mapping_msgs::CollisionMap &collision_map);

    /**
   * \brief Resets the distance field to the max_distance.
   */
    virtual void reset()=0;

    /**
   * \brief Gets the distance to the closest obstacle at the given location.
   */
    double getDistance(double x, double y, double z) const;

    /**
   * \brief Gets the distance at a location and the gradient of the field.
   */
    virtual double getDistanceGradient(double x, double y, double z, double& gradient_x, double& gradient_y, double& gradient_z) const;

    /**
   * \brief Gets the distance to the closest obstacle at the given integer cell location.
   */
    double getDistanceFromCell(int x, int y, int z) const;

    /**
   * \brief Publishes an iso-surface to rviz.
   *
   * Publishes an iso-surface containing points between min_radius and max_radius
   * as visualization markers for rviz.
   */

    // REMOVED
    //  void visualize(double min_radius, double max_radius, std::string frame_id, ros::Time stamp);

protected:
    virtual double getDistance(const T& object) const=0;

private:
    // REMOVED
    //  ros::Publisher pub_viz_;
};

//////////////////////////// template function definitions follow //////////////

template <typename T>
DistanceField<T>::~DistanceField()
{

}

template <typename T>
DistanceField<T>::DistanceField(double size_x, double size_y, double size_z, double resolution,
                                Transform origin_in, T default_object):
    VoxelGrid<T>(size_x, size_y, size_z, resolution, origin_in, default_object)
{
    // REMOVED ROS depedency
    //  ros::NodeHandle node;
    //  pub_viz_ = node.advertise<visualization_msgs::Marker>("visualization_marker", 3);
}

template <typename T>
double DistanceField<T>::getDistance(double x, double y, double z) const
{
    return getDistance((*this)(x,y,z));
}

template <typename T>
double DistanceField<T>::getDistanceFromCell(int x, int y, int z) const
{
    return getDistance(this->getCell(x,y,z));
}

template <typename T>
double DistanceField<T>::getDistanceGradient(double x, double y, double z, double& gradient_x, double& gradient_y, double& gradient_z) const
{
    return 0.0;
}

// template <typename T>
// void DistanceField<T>::visualize(double min_radius, double max_radius, std::string frame_id, ros::Time stamp)
// {
//   visualization_msgs::Marker inf_marker; // Marker for the inflation
//   inf_marker.header.frame_id = frame_id;
//   inf_marker.header.stamp = stamp;
//   inf_marker.ns = "distance_field";
//   inf_marker.id = 1;
//   inf_marker.type = visualization_msgs::Marker::CUBE_LIST;
//   inf_marker.action = 0;
//   inf_marker.scale.x = this->resolution_[VoxelGrid<T>::DIM_X];
//   inf_marker.scale.y = this->resolution_[VoxelGrid<T>::DIM_Y];
//   inf_marker.scale.z = this->resolution_[VoxelGrid<T>::DIM_Z];
//   inf_marker.color.r = 1.0;
//   inf_marker.color.g = 0.0;
//   inf_marker.color.b = 0.0;
//   inf_marker.color.a = 0.1;
//   inf_marker.lifetime = ros::Duration(30.0);
// 
//   inf_marker.points.reserve(100000);
//   for (int x = 0; x < this->num_cells_[VoxelGrid<T>::DIM_X]; ++x)
//   {
//     for (int y = 0; y < this->num_cells_[VoxelGrid<T>::DIM_Y]; ++y)
//     {
//       for (int z = 0; z < this->num_cells_[VoxelGrid<T>::DIM_Z]; ++z)
//       {
//         double dist = getDistanceFromCell(x,y,z);
//         if (dist >= min_radius && dist <= max_radius)
//         {
//           int last = inf_marker.points.size();
//           inf_marker.points.resize(last + 1);
//           this->gridToWorld(x,y,z,
//                       inf_marker.points[last].x,
//                       inf_marker.points[last].y,
//                       inf_marker.points[last].z);
//         }
//       }
//     }
//   }
//   ROS_DEBUG("Publishing markers: %d inflated", inf_marker.points.size());
//   pub_viz_.publish(inf_marker);
// }

// template <typename T>
// void DistanceField<T>::addCollisionMapToField(const mapping_msgs::CollisionMap &collision_map)
// {
//   size_t num_boxes = collision_map.boxes.size();
//   std::vector<point3d> points;
//   points.reserve(num_boxes);
//   for (size_t i=0; i<num_boxes; ++i)
//   {
//     points.push_back(point3d(
//         collision_map.boxes[i].center.x,
//         collision_map.boxes[i].center.y,
//         collision_map.boxes[i].center.z
//         ));
//   }
//   addPointsToField(points);
// }


}
#endif /* DF_DISTANCE_FIELD_H_ */
