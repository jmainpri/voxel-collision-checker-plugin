/*********************************************************************
Cost field -- voxel grid with 3d gradients
costs need to be manually input 
*/
/** \author Lillian Chang November 2009 */

#ifndef COST_FIELD_H_
#define COST_FIELD_H_

#include <voxel_grid.hpp>
#include <vector>
#include <list>

#include <iostream>
#include <fstream>
#include <string>


/* voxel grid parameters */
struct grid3d_pars {    
    int nx,ny,nz; // number of cells
    double sx,sy,sz; // size
    double res;   // resolution
    double ox,oy,oz; // origin location
};



struct Configuration
{
    std::vector<double> joint_values;
};

using namespace distance_field;


/**
 * \brief A VoxelGrid of scalar costs over which a gradient can be computed.
 *
 */
class CostField: public VoxelGrid<double>
{
public:
    /**
   * \brief Constructor for the Cost Field.
   *
   * @param size_x Size (x axis) of the container in meters
   * @param size_y Size (y axis) of the container in meters
   * @param size_z Size (z axis) of the container in meters
   * @param resolution: resolution (size of a single cell) in meters
   * @param origin_in transform of the origin of the container
   * @param default_object The object to return for an out-of-bounds query
   */
    CostField(double size_x, double size_y, double size_z, double resolution,
              Transform origin_in, double default_cost);
    CostField(){}
    virtual ~CostField();

    /**
   * \brief Gets the cost to the closest obstacle at the given location.
   */
    double getCost(double x, double y, double z) const;

    /**
   * \brief Gets the cost at a location and the approximate gradient of the field.
   */
    double getCostGradient(double x, double y, double z, double& gradient_x, double& gradient_y, double& gradient_z) const;

    //  /**
    //   * \brief Gets the cost at a location and the true gradient of the field.
    //   */
    //  double getCostGradientExact(double x, double y, double z, double& gradient_x, double& gradient_y, double& gradient_z) const;


    /**
   * \brief Gets the cost at the given integer cell location.
   */
    double getCostFromCell(int x, int y, int z) const;

    /**
   * \brief Sets the cost at the given integer cell location.
   */
    bool setCostOfCell(int x, int y, int z, double cost);

    grid3d_pars getGridParameters(void) const;

private:
    grid3d_pars grid;

};

//////////////////////////// inline function follow //////////////

inline CostField::~CostField()
{
}

inline CostField::CostField(double size_x, double size_y, double size_z, double resolution,
                            Transform origin_in, double default_cost):
    VoxelGrid<double>(size_x, size_y, size_z, resolution, origin_in, default_cost)
{
    // set the voxel grid parameters
    grid.res = resolution;
    // size
    grid.sx = size_[DIM_X];
    grid.sy = size_[DIM_Y];
    grid.sz = size_[DIM_Z];
    // number of cells
    grid.nx = num_cells_[DIM_X];
    grid.ny = num_cells_[DIM_Y];
    grid.nz = num_cells_[DIM_Z];
    // origin
    grid.ox = origin_.trans[DIM_X];
    grid.oy = origin_.trans[DIM_Y];
    grid.oz = origin_.trans[DIM_Z];
}

inline double CostField::getCost(double x, double y, double z) const
{
    int gx, gy, gz;

    if (worldToGrid(x, y, z, gx, gy, gz)){
        return getCell(gx,gy,gz);
    } else {
        return default_object_;
    }
    //   return getCostFromCell(gx,gy,gz);
}

inline double CostField::getCostFromCell(int x, int y, int z) const
{
    if (isCellValid(x,y,z)) {
        return getCell(x,y,z);
    } else {
        return default_object_;
    }
    //   return (*this)(x,y,z);
}

inline bool CostField::setCostOfCell(int x, int y, int z, double cost) 
{
    if (!isCellValid(x,y,z)) {
        return false;
    } else {
        setCell(x,y,z,cost);
        return true;
    }
    //   return (*this)(x,y,z);
}


inline double CostField::getCostGradient(double x, double y, double z, double& gradient_x, double& gradient_y, double& gradient_z) const
{
    int gx, gy, gz;

    worldToGrid(x, y, z, gx, gy, gz);

    // if out of bounds, return default object cost, and 0 gradient
    // we need extra padding of 1 to get gradients
    if (gx<1 || gy<1 || gz<1 || gx>=num_cells_[DIM_X]-1 || gy>=num_cells_[DIM_Y]-1 || gz>=num_cells_[DIM_Z]-1)
    {
        gradient_x = 0.0;
        gradient_y = 0.0;
        gradient_z = 0.0;
        return default_object_;
    }

    double inv_twice_resolution = 1./(2.*grid.res);
    gradient_x = (getCostFromCell(gx+1,gy,gz) - getCostFromCell(gx-1,gy,gz))*inv_twice_resolution;
    gradient_y = (getCostFromCell(gx,gy+1,gz) - getCostFromCell(gx,gy-1,gz))*inv_twice_resolution;
    gradient_z = (getCostFromCell(gx,gy,gz+1) - getCostFromCell(gx,gy,gz-1))*inv_twice_resolution;

    return getCostFromCell(gx,gy,gz);

}


// inline double CostField::getCostGradientExact(double x, double y, double z, double& gradient_x, double& gradient_y, double& gradient_z) const
// {
//   int gx, gy, gz;
// 
//   worldToGrid(x, y, z, gx, gy, gz);
// 
//   // if out of bounds, return default object cost, and 0 gradient
//   // we need extra padding of 1 to get gradients
//   if (gx<1 || gy<1 || gz<1 || gx>=num_cells_[DIM_X]-1 || gy>=num_cells_[DIM_Y]-1 || gz>=num_cells_[DIM_Z]-1)
//   {
//     gradient_x = 0.0;
//     gradient_y = 0.0;
//     gradient_z = 0.0;
//     return default_object_;
//   }
// 
//   double inv_resolution = 1.0/grid.res;
//   double tempcost;
//   double costdiff = 0;
//   double bestcost = getCostFromCell(gx,gy,gz);
//   int offset[3];
//   offset[0] = 0; offset[1] = 0; offset[2] = 0;
//   for(int i = -1; i < 2; i++)
//       for(int j = -1; j < 2; j++)
//           for(int k = -1; k < 2; k++)
//           {
//             tempcost = getCostFromCell(gx+i,gy+j,gz+k);
//             if(tempcost < bestcost)
//             {
//                 bestcost = tempcost;
//                 costdiff = getCostFromCell(gx,gy,gz) - tempcost;
//                 offset[0] = i; offset[1] = j; offset[2] = k;
//             }
//           }
//   
//   gradient_x = -offset[0];//*costdiff*inv_resolution;
//   gradient_y = -offset[1];//*costdiff*inv_resolution;
//   gradient_z = -offset[2];//*costdiff*inv_resolution;
//  
// 
//   return getCostFromCell(gx,gy,gz);
// 
// }

inline grid3d_pars CostField::getGridParameters(void) const
{
    return ( grid );
}


#endif /* COST_FIELD_H_ */
