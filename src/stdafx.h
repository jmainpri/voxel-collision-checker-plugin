// Copyright (C) 2006-2008 Carnegie Mellon University (rdiankov@cs.cmu.edu)
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

//This file contains includes of necessary headers and several useful macros and functions
#ifndef BIRRT_PLANNER_STDAFX
#define BIRRT_PLANNER_STDAFX

#define _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_DEPRECATE

#include <assert.h>
#include <cstdio>
#include <cmath>
#include <cstdlib>

#include <string.h>
#include <vector>
#include <list>
#include <map>
#include <string>
#include <fstream>
#include <iostream>

using namespace std;
typedef unsigned int u32;

#include <sys/timeb.h>    // ftime(), struct timeb
#include <sys/time.h>

template<class T>
inline T CLAMP_ON_RANGE(T value, T min, T max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

inline unsigned long timeGetTime()
{
#ifdef _WIN32
    _timeb t;
    _ftime(&t);
#else
    timeb t;
    ftime(&t);
#endif

    return (unsigned long)(t.time*1000+t.millitm);
}

inline double timeGetThreadTime()
{
    struct timespec tp;
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &tp);


    return ((double)tp.tv_sec + (((double)tp.tv_nsec)/1000000000.0));
}

inline float RANDOM_FLOAT()
{
#if defined(__IRIX__)
    return drand48();
#else
    return rand()/((float)RAND_MAX);
#endif
}

inline float RANDOM_FLOAT(float maximum)
{
#if defined(__IRIX__)
    return (drand48() * maximum);
#else
    return (RANDOM_FLOAT() * maximum);
#endif
}

inline int RANDOM_INT(int maximum)
{
#if defined(__IRIX__)
    return (random() % maximum);
#else
    return (rand() % maximum);
#endif
}


#define FOREACH(it, v) for(typeof((v).begin()) it = (v).begin(); it != (v).end(); (it)++)

#define FOREACHC FOREACH



#ifndef ARRAYSIZE
#define ARRAYSIZE(x) (sizeof(x)/(sizeof( (x)[0] )))
#endif

#define FORIT(it, v) for(it = (v).begin(); it != (v).end(); (it)++)

#ifdef _WIN32

#define WCSTOK(str, delim, ptr) wcstok(str, delim)

// define wcsicmp for MAC OS X
#elif defined(__APPLE_CC__)

#define WCSTOK(str, delim, ptr) wcstok(str, delim, ptr);

#define strnicmp strncasecmp
#define stricmp strcasecmp

inline int wcsicmp(const wchar_t* s1, const wchar_t* s2)
{
  char str1[128], str2[128];
  sprintf(str1, "%S", s1);
  sprintf(str2, "%S", s2);
  return stricmp(str1, str2);
}


#else

#define WCSTOK(str, delim, ptr) wcstok(str, delim, ptr)

#define strnicmp strncasecmp
#define stricmp strcasecmp
#define wcsnicmp wcsncasecmp
#define wcsicmp wcscasecmp

#endif

inline std::wstring _ravembstowcs(const char* pstr)
{
    size_t len = mbstowcs(NULL, pstr, 0);
    std::wstring w; w.resize(len);
    mbstowcs(&w[0], pstr, len);
    return w;
}



#include <rave/rave.h>
using namespace OpenRAVE;
extern "C"
{
#include <stdio.h>
#include <stdlib.h>
#include <qhull/qhull.h>
#include <qhull/mem.h>
#include <qhull/qset.h>
#include <qhull/geom.h>
#include <qhull/merge.h>
#include <qhull/poly.h>
#include <qhull/io.h>
#include <qhull/stat.h>
}

#include <boost/assert.hpp>
#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <boost/array.hpp>


#include <iostream>
struct null_deleter
{
    void operator()(void const *) const {
    }
};

class DoubleVectorToFloatVector : public vector<float> {
    public:
        DoubleVectorToFloatVector(vector<double> doubleIn)
        {
            resize(doubleIn.size());
            for (size_t i = 0; i < doubleIn.size(); i++)
            {
                at(i) = (float)doubleIn[i];
            }
        }

         DoubleVectorToFloatVector(OpenRAVE::geometry::RaveVector<double>  doubleIn)
        {
            resize(4);
            at(0) = (float)doubleIn.x;
            at(1) = (float)doubleIn.y;
            at(2) = (float)doubleIn.z;
            at(3) = (float)doubleIn.w;
        }

         DoubleVectorToFloatVector(OpenRAVE::geometry::RaveVector<float>  doubleIn)
        {
            resize(4);
            at(0) = (float)doubleIn.x;
            at(1) = (float)doubleIn.y;
            at(2) = (float)doubleIn.z;
            at(3) = (float)doubleIn.w;
        }
};

class RaveDoubleVectorToRaveFloatVector : public RaveVector<float> {
    public:
        RaveDoubleVectorToRaveFloatVector(RaveVector<double> doubleIn)
        {
            x = (float)doubleIn.x;
            y = (float)doubleIn.y;
            z = (float)doubleIn.z;
            w = (float)doubleIn.w;
        }
};

//#include "newmatap.h"
//#include "newmatio.h"
#include <newmat/newmat.h>
#define SETUP_C_SUBSCRIPTS

#include "voxel_grid.hpp"
#include "libDVXL.h"
#include "propagation_distance_field.hpp"
#include "cost_field.hpp"
#include "init_obstacle_field.hpp"
#include "voxelcolcheck.hpp"

#include "dvxlcolcheck.hpp"

//stuff for voxel grid planning (these are just temporary until there is a voxel class for link geometries
//extern std::vector<boost::shared_ptr<ConfigurationCostFunction> > global_vconfigurationcost;


extern std::vector<std::vector<Vector> > global_vvLinkSamples;

// extern VoxelGrid<int> global_vg;
// extern std::vector<boost::shared_ptr<VoxelCollisionChecker> > global_vvoxelcollider;
// extern std::vector<CostField> global_vcostgrid;
           
//stuff for plotting
extern std::vector<GraphHandlePtr> graphptrs;
#endif
