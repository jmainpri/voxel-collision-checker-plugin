#ifndef INIT_OBS_FIELD_H_
#define INIT_OBS_FIELD_H_

#include <iostream>
#include <fstream>
#include <string>
#include "voxel_grid.hpp"
#include <propagation_distance_field.hpp>
#include "cost_field.hpp"

#include "stdafx.h"

#define COMPUTE_NEW_VG         1   //whether to recompute voxel grid or read from file
#define LARGE_DISTANCE      10.0 //meters, large distance to move robot out of the way for collision checking
#define ROBOT_REACH          1.5 //meters, conservative radius to fully describe workspace
#define ROBOT_HEIGHT         1.5 //meters, height of robot to "ground" (below wheels)
#define VOXEL_RES            0.025 //0.05 meters, edge length of a voxel
#define VG_OFFSET           (ROBOT_REACH-VOXEL_RES) // offset for voxel origin
#define VG_FILENAME         "saved_voxelgrid_new.txt"
#define PADDING             0.1 // meters, padding for cost function


//#endif
//#ifdef wafoinwefionwef
using namespace std;
using namespace OpenRAVE;
using namespace distance_field;

double obstacleCost( double distance);

VoxelGrid<int> createVoxelGrid(int compute_new_vg, EnvironmentBasePtr penv, RobotBasePtr robot, std::vector<KinBodyPtr>& colbodies );

VoxelGrid<int> createEmptyVoxelGrid(RobotBasePtr robot);


PropagationDistanceField createPDFfromVoxelGrid( VoxelGrid<int>* vg );

CostField  createCostFieldfromVoxelGrid( VoxelGrid<int>* vg );



#endif /* INIT_OBS_FIELD_H */

