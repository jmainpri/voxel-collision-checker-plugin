#ifndef INIT_OBS_FIELD_H_
#define INIT_OBS_FIELD_H_

#include <iostream>
#include <fstream>
#include <string>
#include "voxel_grid.hpp"
#include "propagation_distance_field.hpp"
#include "cost_field.hpp"
#include "collision_point.hpp"

#include "stdafx.h"

#define COMPUTE_NEW_VG         1   //whether to recompute voxel grid or read from file
#define LARGE_DISTANCE      10.0 //meters, large distance to move robot out of the way for collision checking
#define ROBOT_REACH          1.5 //meters, conservative radius to fully describe workspace
#define ROBOT_HEIGHT         1.5 //meters, height of robot to "ground" (below wheels)
#define VOXEL_RES            0.025 //0.05 meters, edge length of a voxel
#define VG_OFFSET           (ROBOT_REACH-VOXEL_RES) // offset for voxel origin
#define VG_FILENAME         "saved_voxelgrid_new.txt"
#define PADDING             0.1 // meters, padding for cost function


double obstacleCost( double distance);

distance_field::VoxelGrid<int> createVoxelGrid( int compute_new_vg, OpenRAVE::EnvironmentBasePtr penv, OpenRAVE::RobotBasePtr robot, std::vector<OpenRAVE::KinBodyPtr>& colbodies );

distance_field::VoxelGrid<int> createEmptyVoxelGrid( OpenRAVE::RobotBasePtr robot );

PropagationDistanceField createPDFfromVoxelGrid( const distance_field::VoxelGrid<int>& vg, OpenRAVE::EnvironmentBasePtr penv, std::vector< boost::shared_ptr<void> >& graphptr );

CostField createCostFieldfromVoxelGrid( const distance_field::VoxelGrid<int>& vg );

std::vector<CollisionPoint> createCollionPointsForPr2( OpenRAVE::RobotBasePtr body );

#endif /* INIT_OBS_FIELD_H */
