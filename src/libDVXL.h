#ifndef LIBDVXL_H
#define LIBDVXL_H

#include "stdlib.h"
#include "stdio.h"
#include <string>
#include "string.h"
#include <iostream>
#include <algorithm>
#include <vector>
#include <rave/rave.h>
#include <exception>
#include <stdexcept>
#include "voxel_grid.hpp"

typedef struct dvxl
{
    float deformability;
    float sensitivity;
    uint8_t R;
    uint8_t G;
    uint8_t B;
    uint8_t A;
} dvxl_t;

class DVXL_GRID
{
protected:

    void LoadLegacyFiletype(std::string filename, double legacy_resolution)
    {
        FILE *array_file = fopen(filename.c_str(), "r");
        if (array_file == NULL)
        {
            throw std::invalid_argument("DVXL file is damaged or corrupt and could not be read!");
        }
        //Read file header
        int Xdim, Ydim, Zdim, Xcenter, Ycenter, Zcenter;
        float defCost, defDeform;
        int read = fscanf(array_file, "D-Voxel Array Storage File\nSize: %d|%d|%d\nCenter: %d|%d|%d\nDefaults: %f|%f\nDATA", &Xdim, &Ydim, &Zdim, &Xcenter, &Ycenter, &Zcenter, &defCost, &defDeform);
        if (read != 8)
        {
            throw std::invalid_argument("DVXL file is damaged or corrupt and could not be read!");
        }
        //Make an empty voxelgrid
        default_deformability_ = defDeform;
        default_sensitivity_ = defCost;
        dvxl_t out_of_bounds_dvxl;
        out_of_bounds_dvxl.deformability = 0.0;
        out_of_bounds_dvxl.sensitivity = 0.0;
        out_of_bounds_dvxl.R = 0;
        out_of_bounds_dvxl.G = 0;
        out_of_bounds_dvxl.B = 0;
        out_of_bounds_dvxl.A = 0;
        OpenRAVE::Transform origin;
        origin.trans.x = (double) -Xcenter;
        origin.trans.y = (double) -Ycenter;
        origin.trans.z = (double) -Zcenter;
        grid_ = distance_field::VoxelGrid<dvxl_t>((double) Xdim * legacy_resolution, (double) Ydim * legacy_resolution, (double) Zdim * legacy_resolution, legacy_resolution, origin, out_of_bounds_dvxl);
        SetValues(default_deformability_, default_sensitivity_, 0, 0, 0, 0);
        //Read file data
        char * temp_read = (char *) malloc(sizeof(dvxl_t));
        for (int i = 0; i < Xdim; i++)
        {
            for (int j = 0; j < Ydim; j++)
            {
                for (int k = 0; k < Zdim; k++)
                {
                    int read = fread(temp_read, sizeof(dvxl_t), 1, array_file);
                    if (read != 1)
                    {
                        throw std::invalid_argument("DVXL file is damaged or corrupt and could not be read!");
                    }
                    dvxl_t temp_dvxl;
                    temp_dvxl.deformability = *( (float*)(temp_read) );
                    temp_dvxl.sensitivity = *( (float*)(temp_read + 4) );
                    temp_dvxl.R = temp_read[8];
                    temp_dvxl.G = temp_read[9];
                    temp_dvxl.B = temp_read[10];
                    temp_dvxl.A = temp_read[11];
                    grid_.setCell(i, j, k, temp_dvxl);
                }
            }
        }
        //Flush & close file
        fclose(array_file);
        free(temp_read);
    }

    void LoadCurrentFiletype(std::string filename)
    {
        FILE *array_file = fopen(filename.c_str(), "r");
        if (array_file == NULL)
        {
            throw std::invalid_argument("DVXL file is damaged or corrupt and could not be read!");
        }
        //Read file header
        int Xdim, Ydim, Zdim;
        double xSize, ySize, zSize, xRes, yRes, zRes, xT, yT, zT, xR, yR, zR, wR;
        int read = fscanf(array_file, "Second-generation DVXL Grid Storage File\nSize: %lf|%lf|%lf\nResolution: %lf|%lf|%lf\nElements: %d|%d|%d\nOrigin transform: %lf|%lf|%lf||%lf|%lf|%lf|%lf\nDefaults: %f|%f\nDATA", &xSize, &ySize, &zSize, &xRes, &yRes, &zRes, &Xdim, &Ydim, &Zdim, &xT, &yT, &zT, &xR, &yR, &zR, &wR, &default_sensitivity_, &default_deformability_);
        if (read != 18)
        {
            throw std::invalid_argument("DVXL file is damaged or corrupt and could not be read!");
        }
        //Make an empty voxelgrid
        dvxl_t out_of_bounds_dvxl;
        out_of_bounds_dvxl.deformability = 0.0;
        out_of_bounds_dvxl.sensitivity = 0.0;
        out_of_bounds_dvxl.R = 0;
        out_of_bounds_dvxl.G = 0;
        out_of_bounds_dvxl.B = 0;
        out_of_bounds_dvxl.A = 0;
        OpenRAVE::Transform origin;
        origin.trans.x = xT;
        origin.trans.y = yT;
        origin.trans.z = zT;
        origin.rot.x = xR;
        origin.rot.y = yR;
        origin.rot.z = zR;
        origin.rot.w = wR;
        grid_ = distance_field::VoxelGrid<dvxl_t>(xSize, ySize, zSize, xRes, origin, out_of_bounds_dvxl);
        SetValues(default_deformability_, default_sensitivity_, 0, 0, 0, 0);
        //Read file data
        char * temp_read = (char *) malloc(sizeof(dvxl_t));
        for (int i = 0; i < Xdim; i++)
        {
            for (int j = 0; j < Ydim; j++)
            {
                for (int k = 0; k < Zdim; k++)
                {
                    int read = fread(temp_read, sizeof(dvxl_t), 1, array_file);
                    if (read != 1)
                    {
                        throw std::invalid_argument("DVXL file is damaged or corrupt and could not be read!");
                    }
                    dvxl_t temp_dvxl;
                    temp_dvxl.deformability = *( (float*)(temp_read) );
                    temp_dvxl.sensitivity = *( (float*)(temp_read + 4) );
                    temp_dvxl.R = temp_read[8];
                    temp_dvxl.G = temp_read[9];
                    temp_dvxl.B = temp_read[10];
                    temp_dvxl.A = temp_read[11];
                    grid_.setCell(i, j, k, temp_dvxl);
                }
            }
        }
        //Flush & close file
        fclose(array_file);
        free(temp_read);
    }

    void SetValues(float deformability, float sensitivity, uint8_t R, uint8_t G, uint8_t B, uint8_t A)
    {
        dvxl_t default_dvxl;
        default_dvxl.deformability = deformability;
        default_dvxl.sensitivity = sensitivity;
        default_dvxl.R = R;
        default_dvxl.G = G;
        default_dvxl.B = B;
        default_dvxl.A = A;
        grid_.reset(default_dvxl);
    }

    bool WriteLegacyFiletype(std::string filename)
    {
        FILE *array_file = fopen(filename.c_str(), "w");
        //Write file header
        int xDim = grid_.getNumCells(grid_.DIM_X);
        int yDim = grid_.getNumCells(grid_.DIM_Y);
        int zDim = grid_.getNumCells(grid_.DIM_Z);
        fprintf(array_file, "D-Voxel Array Storage File\nSize: %d|%d|%d\nCenter: %d|%d|%d\nDefaults: %f|%f\nDATA", xDim, yDim, zDim, 0, 0, 0, default_sensitivity_, default_deformability_);
        //Write file data
        for (int i = 0; i < xDim; i++)
        {
            for (int j = 0; j < yDim; j++)
            {
                for (int k = 0; k < zDim; k++)
                {
                    dvxl_t temp = grid_.getCell(i, j, k);
                    fwrite(&temp, sizeof(dvxl_t), 1, array_file);
                }
            }
        }
        //Flush & close file
        int result = fclose(array_file);
        if (result == 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    bool WriteCurrentFiletype(std::string filename)
    {
        FILE *array_file = fopen(filename.c_str(), "w");
        //Write file header
        int xDim = grid_.getNumCells(grid_.DIM_X);
        int yDim = grid_.getNumCells(grid_.DIM_Y);
        int zDim = grid_.getNumCells(grid_.DIM_Z);
        double xSize = grid_.getSize(grid_.DIM_X);
        double ySize = grid_.getSize(grid_.DIM_Y);
        double zSize = grid_.getSize(grid_.DIM_Z);
        double xRes = grid_.getResolution(grid_.DIM_X);
        double yRes = grid_.getResolution(grid_.DIM_Y);
        double zRes = grid_.getResolution(grid_.DIM_Z);
        OpenRAVE::Transform origin = grid_.getOriginTransform();
        double xT = origin.trans.x;
        double yT = origin.trans.y;
        double zT = origin.trans.z;
        double xR = origin.rot.x;
        double yR = origin.rot.y;
        double zR = origin.rot.z;
        double wR = origin.rot.w;
        fprintf(array_file, "Second-generation DVXL Grid Storage File\nSize: %lf|%lf|%lf\nResolution: %lf|%lf|%lf\nElements: %d|%d|%d\nOrigin transform: %lf|%lf|%lf||%lf|%lf|%lf|%lf\nDefaults: %f|%f\nDATA", xSize, ySize, zSize, xRes, yRes, zRes, xDim, yDim, zDim, xT, yT, zT, xR, yR, zR, wR, default_sensitivity_, default_deformability_);
        //Write file data
        for (int i = 0; i < xDim; i++)
        {
            for (int j = 0; j < yDim; j++)
            {
                for (int k = 0; k < zDim; k++)
                {
                    dvxl_t temp = grid_.getCell(i, j, k);
                    fwrite(&temp, sizeof(dvxl_t), 1, array_file);
                }
            }
        }
        //Flush & close file
        int result = fclose(array_file);
        if (result == 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    float default_deformability_;

    float default_sensitivity_;

public:

    DVXL_GRID(std::string filename, double legacy_resolution=-1.0)
    {
        if (legacy_resolution == -1.0)
        {
            LoadCurrentFiletype(filename);
        }
        else
        {
            LoadLegacyFiletype(filename, legacy_resolution);
        }
    }

    DVXL_GRID(double Xdim, double Ydim, double Zdim, double resolution, float default_deformability, float default_sensitivity, OpenRAVE::Transform origin)
    {
        default_deformability_ = default_deformability;
        default_sensitivity_ = default_sensitivity;
        dvxl_t out_of_bounds_dvxl;
        out_of_bounds_dvxl.deformability = 0.0;
        out_of_bounds_dvxl.sensitivity = 0.0;
        out_of_bounds_dvxl.R = 0;
        out_of_bounds_dvxl.G = 0;
        out_of_bounds_dvxl.B = 0;
        out_of_bounds_dvxl.A = 0;
        grid_ = distance_field::VoxelGrid<dvxl_t>(Xdim, Ydim, Zdim, resolution, origin, out_of_bounds_dvxl);
        SetValues(default_deformability_, default_sensitivity_, 0, 0, 0, 0);
    }

    ~DVXL_GRID()
    {
    }

    bool ConvertToLegacyFiletype(std::string filename)
    {
        return WriteLegacyFiletype(filename);
    }

    std::vector<float> GetDefaults()
    {
        std::vector<float> defaults;
        defaults.resize(2);
        defaults[0] = default_deformability_;
        defaults[1] = default_sensitivity_;
        return defaults;
    }

    void SetNewDefaults(float deformability, float sensitivity)
    {
        default_deformability_ = deformability;
        default_sensitivity_ = sensitivity;
    }

    void Reset()
    {
        SetValues(default_deformability_, default_sensitivity_, 0, 0, 0, 0);
    }

    bool Save(std::string filename)
    {
        return WriteCurrentFiletype(filename);
    }

    distance_field::VoxelGrid<dvxl_t> grid_;
};

#endif // LIBDVXL_H
