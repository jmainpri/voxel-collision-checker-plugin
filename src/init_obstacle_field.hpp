#ifndef INIT_OBS_FIELD_H_
#define INIT_OBS_FIELD_H_

#include <iostream>
#include <fstream>
#include <string>

#include "stdafx.h"

#include "voxel_grid.hpp"
#include "propagation_distance_field.hpp"
//#include "cost_field.hpp"
#include "collision_point.hpp"

#define COMPUTE_NEW_VG         1   //whether to recompute voxel grid or read from file
#define LARGE_DISTANCE      10.0 //meters, large distance to move robot out of the way for collision checking
#define ROBOT_REACH          1.5 //meters, conservative radius to fully describe workspace
#define ROBOT_HEIGHT         1.5 //meters, height of robot to "ground" (below wheels)
// #define VOXEL_RES            0.025 //0.05 meters, edge length of a voxel
#define VG_OFFSET           (ROBOT_REACH-VOXEL_RES) // offset for voxel origin
#define VG_FILENAME         "saved_voxelgrid_new.txt"
#define PADDING             0.1 // meters, padding for cost function


double obstacleCost( double distance);

// Drawing
void drawClearHandles();
void setDrawingDistance( double dist, double blue_threshold = 0.3 );
void setVoxelGridSize(double x, double y, double z, double res, Transform grid_origin = Transform() );

// Voxel Grid
distance_field::VoxelGrid<int> createVoxelGrid( int compute_new_vg, OpenRAVE::EnvironmentBasePtr penv, OpenRAVE::RobotBasePtr robot, std::vector<OpenRAVE::KinBodyPtr>& colbodies );
distance_field::VoxelGrid<int> createEmptyVoxelGrid( OpenRAVE::RobotBasePtr robot );

// Propagation distance field
distance_field::PropagationDistanceField createPDFfromVoxelGrid( const distance_field::VoxelGrid<int>& vg, OpenRAVE::EnvironmentBasePtr penv );
void drawPDF( const distance_field::PropagationDistanceField& PDF, OpenRAVE::EnvironmentBasePtr penv );

// Collision points
std::vector<distance_field::CollisionPoint> createCollionPointsForRobot( OpenRAVE::RobotBasePtr robot, const std::vector<float>& radii, bool use_second_joint=false );

#endif /* INIT_OBS_FIELD_H */
