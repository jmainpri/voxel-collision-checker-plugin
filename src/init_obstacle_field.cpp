#include "init_obstacle_field.hpp"

using namespace std;
using namespace OpenRAVE;
using namespace distance_field;

string convertInt(int number)
{
   stringstream ss;//create a stringstream
   ss << number;//add number to the stream
   return ss.str();//return a string with the contents of the stream
}

/**
 * Computes the color gradiant
 * color: the output vector
 * x: the gradiant (beetween 0 and 360)
 * min and max: variation of the RGB channels
 */
void GroundColorMix( std::vector<float>& color, double x, double min, double max)
{
    /*
     * Red = 0
     * Green = 1
     * Blue = 2
     */
    double posSlope = (max-min)/60;
    double negSlope = (min-max)/60;

    if( x < 60 )
    {
        color[0] = max;
        color[1] = posSlope*x+min;
        color[2] = min;
        return;
    }
    else if ( x < 120 )
    {
        color[0] = negSlope*x+2*max+min;
        color[1] = max;
        color[2] = min;
        return;
    }
    else if ( x < 180  )
    {
        color[0] = min;
        color[1] = max;
        color[2] = posSlope*x-2*max+min;
        return;
    }
    else if ( x < 240  )
    {
        color[0] = min;
        color[1] = negSlope*x+4*max+min;
        color[2] = max;
        return;
    }
    else if ( x < 300  )
    {
        color[0] = posSlope*x-4*max+min;
        color[1] = min;
        color[2] = max;
        return;
    }
    else
    {
        color[0] = max;
        color[1] = min;
        color[2] = negSlope*x+6*max;
        return;
    }
}

/**
 * Computes the color gradiant
 * color: the output vector
 * x: the gradiant beetween 0 (Green) and 1 (Red)
 * min and max: variation of the RGB channels (Move3D 0 -> 1)
 */
void GroundColorMixGreenToRed( std::vector<float>& color, double s )
{
    if (s>1) {
        s = 1;
    }
    if (s<0) {
        s=0;
    }
    GroundColorMix(color, 180*(1 - s), 0, 1);
}

static double grid_size_x = 2.0;
static double grid_size_y = 4.0;
static double grid_size_z = 0.5;

static double VOXEL_RES = 0.025;

static Transform grid_origin;

static double draw_distance = 0.7;
static double draw_blue_threshold = 0.7;

static std::vector< boost::shared_ptr<void> > graphptr;

void drawClearHandles()
{
    graphptr.clear();
}

void setDrawingDistance( double dist, double threshold )
{
    draw_distance = dist;
    draw_blue_threshold = threshold;
}

void setVoxelGridSize(double x, double y, double z, double res, Transform origin )
{
    grid_size_x = x;
    grid_size_y = y;
    grid_size_z = z;
    VOXEL_RES = res;
    grid_origin = origin;
}

VoxelGrid<int> createEmptyVoxelGrid(RobotBasePtr robot)
{
    // center the grid around the robot and limit to volume in robot's reach (arm length ~1.5m)
    int OOB = -100; // out of bounds query return
    OpenRAVE::Transform robotT; // robot position
    // robotT = robot->GetLinks()[1]->GetTransform();
    // robotT = robot->GetTransform();

    Transform origin = grid_origin;
    // Transform Toffset;
    // Toffset.trans.x = -1;
    // Toffset.trans.y = -1.5;
    // Toffset.trans.z = 1.3;//-(ROBOT_HEIGHT-VOXEL_RES);

    // Toffset.trans.x = -VG_OFFSET/2;
    // Toffset.trans.y = -VG_OFFSET/1.25-0.1;
    // Toffset.trans.z = 0;//-(ROBOT_HEIGHT-VOXEL_RES);

    // WARNING REMOVE TO HAVE ROBOT CENTERED GRID
    // robotT = origin;

    // origin = robotT*Toffset;
    // origin.rot = robotT.rot;
    // origin.trans.x = robotT.trans.x-VG_OFFSET;
    // origin.trans.y = robotT.trans.y-VG_OFFSET;
    // origin.trans.z = robotT.trans.z-(ROBOT_HEIGHT-VOXEL_RES);
    // 2.0 2
    VoxelGrid<int> vg( grid_size_x, grid_size_y, grid_size_z, VOXEL_RES, origin, OOB );
    vg.reset(0);

    return vg;
}

VoxelGrid<int> createVoxelGrid(int compute_new_vg, EnvironmentBasePtr penv, RobotBasePtr robot, std::vector<KinBodyPtr>& colbodies)
{
    std::vector<KinBodyPtr> allbodies;
    std::vector<KinBodyConstPtr> vbodyexcluded;
    std::vector<KinBody::LinkConstPtr> vlinkexcluded;

    penv->GetBodies(allbodies);
    
    for(size_t i = 0; i < allbodies.size(); i++)
    {
        bool bExclude = true;
        for(size_t j = 0; j < colbodies.size();j++)
        {
            
            if( allbodies[i]->GetName() == colbodies[j]->GetName() )
            {
                bExclude = false;
                break;
            }
        }
        if(bExclude)
            vbodyexcluded.push_back(allbodies[i]);
    }

    VoxelGrid<int> vg = createEmptyVoxelGrid(robot);

    int numX = vg.getNumCells(VoxelGrid<int>::DIM_X);
    int numY = vg.getNumCells(VoxelGrid<int>::DIM_Y);
    int numZ = vg.getNumCells(VoxelGrid<int>::DIM_Z);

    double oX = vg.getOrigin(VoxelGrid<int>::DIM_X);
    double oY = vg.getOrigin(VoxelGrid<int>::DIM_Y);
    double oZ = vg.getOrigin(VoxelGrid<int>::DIM_Z);

    RAVELOG_INFOA("Voxel Grid size %dx%dx%d cells, origin (%f,%f,%f)\n", numX, numY, numZ, oX, oY, oZ);

    //if (COMPUTE_NEW_VG) { // collision check and write to file
    if (compute_new_vg) { // collision check and write to file
        // brute force collision check each grid of environment
        // load a unit volume for checking collisions
        OpenRAVE::KinBodyPtr unitCollision = RaveCreateKinBody(penv);
        // unitCollision->Init("objects/misc/chomp_grid_unit.kinbody.xml", NULL);
        // make a vector of a single AABB to represent the grid unit
        std::vector<AABB> vaabbs;
        AABB box;
        Vector unitExtents;
        unitExtents.x = VOXEL_RES/2.;
        unitExtents.y = VOXEL_RES/2.;
        unitExtents.z = VOXEL_RES/2.;
        box.extents = unitExtents;
        vaabbs.push_back(box);
        
        unitCollision->SetName("v");
        unitCollision->InitFromBoxes(vaabbs, 1);
        //penv->LockPhysics(true);
        penv->Add( unitCollision ); // old add kin body
        //penv->LockPhysics(false);
        OpenRAVE::Transform voxelT; // transform for unit collision, robot
        voxelT.identity();

        // open file for writing
        ofstream vgfile (VG_FILENAME);
        if (!vgfile.is_open()) {
            cout << "Unable to open file";
        } else {
            vgfile << "Voxel Grid " << numX << "x" << numY << "x" << numZ << " cells \n";
            vgfile << "Origin (" << oX << ", " << oY << ", " << oZ << ")\n";
        }
        // check each voxel box
        // first "remove" robot by displacing vertical distance
        Transform robotT = robot->GetTransform();
        robotT.trans.z -= LARGE_DISTANCE;
        robot->SetTransform(robotT);
        int inCollision, numCollisions=0;
        // double posn[3];
        for (int x=0; x<numX; x++)
            for (int y=0; y<numY; y++)
                for (int z=0; z<numZ; z++)
                {
                    // vg.gridToWorld(x,y,z, posn[0], posn[1],posn[2]sdf);
                    // voxelT.trans = posn; // set collision sphere to voxel position
                    vg.gridToWorldTransform( x, y ,z, voxelT );
                    unitCollision->SetTransform(voxelT);
                    if(colbodies.size() == 0)
                        inCollision = penv->CheckCollision( unitCollision );
                    else
                        inCollision = penv->CheckCollision( unitCollision, vbodyexcluded, vlinkexcluded);

                    if (inCollision) { // fill the voxel if there is a collision
                        vg.getCell(x,y,z) = 1;
                        numCollisions++;
                    }
                    // RAVELOG_INFOA("vg(%f, %f, %f) = %d\n", posn[0], posn[1],posn[2], vg.getCell(x,y,z));

                    // write to file
                    vgfile << vg.getCell(x,y,z) << "\n";
                }
        RAVELOG_INFOA("done collision checking, %d collisions (%f percent)\n", numCollisions, (double)numCollisions/(numX*numY*numZ));
        // add robot back to environment
        robotT.trans.z += LARGE_DISTANCE;
        robot->SetTransform(robotT);
        // remove collision check box
        //penv->LockPhysics(true);
        penv->Remove(unitCollision);
        //penv->LockPhysics(false);

        // close file
        vgfile.close();
        
    } else { // read in precomputed voxel grid from file
        
        ifstream vgfile (VG_FILENAME);
        string line;
        if (!vgfile.is_open()) {
            cout << "Unable to open file";
        } else {
            for (int i=0; i < 2; i++) {  // read/skip the header lines
                if (! vgfile.eof() ) {
                    getline (vgfile,line);
                    cout << line << endl;
                }
            }
            // read in each voxel box
            int inCollision, numCollisions=0;
            for (int x=0; x<numX; x++)
                for (int y=0; y<numY; y++)
                    for (int z=0; z<numZ; z++)
                    {
                        vgfile >> inCollision;
                        vg.getCell(x,y,z) = inCollision; // fill the voxel if there is a collision
                        numCollisions += inCollision;
                    }
            RAVELOG_INFOA("done reading in voxel grid, %d collisions, (%f percent)\n", numCollisions, (double)numCollisions/(numX*numY*numZ));
            vgfile.close();
        }
    }

    // Visualize voxel grid and bounds
#if 0
    std::vector<AABB> vaabbs;
    AABB box;
    Vector unitExtents;
    unitExtents.x = VOXEL_RES/2.;
    unitExtents.y = VOXEL_RES/2.;
    unitExtents.z = VOXEL_RES/2.;
    box.extents = unitExtents;
    vaabbs.push_back(box);

    OpenRAVE::Transform voxelT; // transform for unit collision, robot
    double posn[3];

    cout << "numX : " << numX << endl;
    cout << "numY : " << numY << endl;
    cout << "numZ : " << numZ << endl;

    std::vector<OpenRAVE::KinBodyPtr> voxels;

    for (int x=0; x<numX; x++){
        for (int y=0; y<numY; y++)
            for (int z=0; z<numZ; z++)
            {
                // #if 1 // show all voxels
                int isBound = 0;

                if (x==0 || x==numX-1) isBound++;
                if (y==0 || y==numY-1) isBound++;
                if (z==0 || z==numZ-1) isBound++;

                if( vg.getCell(x,y,z) || isBound > 2 ) // voxel if collision or an edge of grid
                {
                    vg.gridToWorld( x, y, z, posn[0], posn[1], posn[2] );
                    voxelT.trans = posn; // set collision sphere to voxel position
                    // penv->LockPhysics(true);
                    OpenRAVE::KinBodyPtr viz_body = RaveCreateKinBody(penv);
                    viz_body->SetName( string("v") + convertInt(x) + "_" + convertInt(y) + "_" + convertInt(z) );
                    viz_body->InitFromBoxes( vaabbs, 1 );
                    viz_body->Enable( false );

                    // penv->Add( viz_body );
                    // penv->LockPhysics(false);
                    viz_body->SetTransform( voxelT );

                    voxels.push_back( viz_body );
                }
                //#endif
                // cout << "cell : " << x << " " << y << " " << z << endl;
            }
        // cout << "cell : " << x << endl;
    }

    // SLOW ...
    for (size_t i=0; i<voxels.size(); i++)
    {
        penv->Add( voxels[i] );
    }
#endif
    
    RAVELOG_INFOA("finished initializing voxel grid\n");

    return(vg);
}

void drawPDF( const PropagationDistanceField& PDF, EnvironmentBasePtr penv )
{
    double res = PDF.getResolution(VoxelGrid<distance_field::PropDistanceFieldVoxel>::DIM_X); // uniform resolution for X,Y,Z

    cout << "resolution : " << res << endl;

    int numX = PDF.getNumCells(VoxelGrid<distance_field::PropDistanceFieldVoxel>::DIM_X);
    int numY = PDF.getNumCells(VoxelGrid<distance_field::PropDistanceFieldVoxel>::DIM_Y);
    int numZ = PDF.getNumCells(VoxelGrid<distance_field::PropDistanceFieldVoxel>::DIM_Z);

    double posn[3];
    std::vector<float> vcolors(3);
    bool inverse = true;

    std::vector<float> x_slices(2);
    x_slices[0] = 20;
    x_slices[1] = 60;

    // for (int i=0; i<int(x_slices.size()); i++){
    //int x = x_slices[i];
    for (int x=0; x<numX; x++){
        for (int y=0; y<numY; y++)
            for (int z=0; z<numZ; z++)
            {
                // int z = 5; // Draw one slice

                // #if 1 // show all voxels
                int isBound = 0;

                if (x==0 || x==numX-1) isBound++;
                if (y==0 || y==numY-1) isBound++;
                if (z==0 || z==numZ-1) isBound++;

                double distance_obst = PDF.getDistance( PDF.getCell(x,y,z) );

                if( distance_obst < draw_distance  || isBound > 2 ) // voxel if collision or an edge of grid
                {
                    PDF.gridToWorld( x, y, z, posn[0], posn[1], posn[2] );

                    double alpha = distance_obst / draw_blue_threshold; // / 10 ;

                    if ( alpha < 0.0 )
                    { alpha = 0.0; }

                    if ( alpha > 1.0 )
                    { alpha = 1.0; }

                    if ( inverse )
                    { alpha = 1 - alpha; }

                    GroundColorMixGreenToRed( vcolors, alpha );

                    std::vector<OpenRAVE::RaveVector<float> > vpoints;
                    OpenRAVE::RaveVector<float> pnt( posn[0], posn[1], posn[2] );
                    vpoints.push_back( pnt );

                    graphptr.push_back( penv->plot3( &vpoints[0].x, vpoints.size(), sizeof( vpoints[0]), res/2, &vcolors[0], 1 ) );
                }
                //#endif
                // cout << "cell : " << x << " " << y << " " << z << endl;
            }
        // cout << "cell : " << x << endl;
    }
}

// PropagationDistanceField -- development code for testing
PropagationDistanceField createPDFfromVoxelGrid( const VoxelGrid<int>& vg, EnvironmentBasePtr penv )
{
    // initialize a distance field based on the voxel grid size
    double sizeX = vg.getSize(VoxelGrid<int>::DIM_X);
    double sizeY = vg.getSize(VoxelGrid<int>::DIM_Y);
    double sizeZ = vg.getSize(VoxelGrid<int>::DIM_Z);
    // double oX = vg.getOrigin(VoxelGrid<int>::DIM_X);
    // double oY = vg.getOrigin(VoxelGrid<int>::DIM_Y);
    // double oZ = vg.getOrigin(VoxelGrid<int>::DIM_Z);
    double res = vg.getResolution(VoxelGrid<int>::DIM_X); // uniform resolution for X,Y,Z
    double maxD = (sizeX + sizeY + sizeZ); // max distance is the diagonal of the grid; consersative bound

    PropagationDistanceField PDF = PropagationDistanceField( sizeX, sizeY, sizeZ, res, vg.getOriginTransform(), maxD);
    cout << "maxD " << maxD << "\n";

    PDF.reset();
    // take collision voxels from voxel grid to fill the distance field
    int numX = vg.getNumCells(VoxelGrid<int>::DIM_X);
    int numY = vg.getNumCells(VoxelGrid<int>::DIM_Y);
    int numZ = vg.getNumCells(VoxelGrid<int>::DIM_Z);

    std::vector<point3d> obstacles;
    point3d voxelPosn;

    for (int x=0; x<numX; x++)
        for (int y=0; y<numY; y++)
            for (int z=0; z<numZ; z++)
            {
                if(vg.getCell(x,y,z)) {   // voxel if collision o
                    vg.gridToWorld(x,y,z, voxelPosn.x, voxelPosn.y, voxelPosn.z);
                    obstacles.push_back(voxelPosn);
                }
            }

    PDF.addPointsToField( obstacles );
    
    // check values

    cout << numX << "x" << numY << "x" << numZ << " cells in VG\n";
//    for (int z=0; z<numZ; z+= 1) {
//        for (int x=0; x<numX; x += 1) {
//            for (int y=0; y<numY; y+= 1)
//            {
//                cout << vg.getCell(x,y,z) << " ";
//            }
//            cout << "\n";
//        }
//        cout << "\n";
//    }

    cout << sizeX << "x" << sizeY << "x" << sizeZ << " m^3 in DF\n";
    /*    for (double z=0; z<sizeZ; z+= 1*res) {
        for (double x=0; x<sizeX; x += 1*res) {
            for (double y=0; y<sizeY; y+= 1*res)*/
//    for (int z=0; z<numZ; z+= 1) {
//        for (int x=0; x<numX; x += 1) {
//            for (int y=0; y<numY; y+= 1)
//            {
//                // cout << setiosflags(ios::fixed) << setprecision(2) << PDF.getDistanceFromCell(x+oX,y+oY,z+oZ) << " ";
//                cout << setiosflags(ios::fixed) << setprecision(2) << PDF.getDistanceFromCell(x,y,z) << " ";

//            }
//            cout << "\n";
//        }
//        cout << "\n";
//    }

    return(PDF);
}

/* Create signed distance field from a voxel grid
Signed distance field is the difference between the Euclidean distance field (EDT) of the voxel grid and the EDT of the voxel grid complement
*/

/*
CostField  createCostFieldfromVoxelGrid( const VoxelGrid<int>& vg )
{
    // initialize a distance field based on the voxel grid size
    double sizeX = vg.getSize(VoxelGrid<int>::DIM_X);
    double sizeY = vg.getSize(VoxelGrid<int>::DIM_Y);
    double sizeZ = vg.getSize(VoxelGrid<int>::DIM_Z);
    // double oX = vg.getOrigin(VoxelGrid<int>::DIM_X);
    // double oY = vg.getOrigin(VoxelGrid<int>::DIM_Y);
    // double oZ = vg.getOrigin(VoxelGrid<int>::DIM_Z);
    double res = vg.getResolution(VoxelGrid<int>::DIM_X); // uniform resolution for X,Y,Z
    double maxD = (sizeX + sizeY + sizeZ); // max distance is the diagonal of the grid; conservative bound
    
    PropagationDistanceField EDT = PropagationDistanceField( sizeX, sizeY, sizeZ, res, vg.getOriginTransform(), maxD);
    PropagationDistanceField EDTnot = PropagationDistanceField( sizeX, sizeY, sizeZ, res, vg.getOriginTransform(), maxD);

    // compute Euclidean distance field for both voxel grid and its complement
    // queue collision voxels for voxel grid and complement
    int numX = vg.getNumCells(VoxelGrid<int>::DIM_X);
    int numY = vg.getNumCells(VoxelGrid<int>::DIM_Y);
    int numZ = vg.getNumCells(VoxelGrid<int>::DIM_Z);

    std::vector<point3d> obstacles, freespace;
    point3d voxelPosn;

    for (int x=0; x<numX; x++)
        for (int y=0; y<numY; y++)
            for (int z=0; z<numZ; z++)
            {
                vg.gridToWorld(x,y,z, voxelPosn.x, voxelPosn.y, voxelPosn.z);
                if(vg.getCell(x,y,z)) {   // voxel if collision
                    obstacles.push_back(voxelPosn);
                } else {
                    freespace.push_back(voxelPosn);
                }
            }

    // generate Euclidean distance field
    EDT.reset();
    EDTnot.reset();
    EDT.addPointsToField( obstacles );
    EDTnot.addPointsToField( freespace ); // complement

    CostField SDF = CostField( sizeX, sizeY, sizeZ, res, vg.getOriginTransform(), 2*maxD);
    // SDF.reset(0.);
    grid3d_pars SDFg = SDF.getGridParameters();

    // point3d posn;
    double distance;
    SDF.reset( 0 );
    for (int x=0; x<SDFg.nx; x++) {
        for (int y=0; y<SDFg.ny; y++) {
            for (int z=0; z<SDFg.nz; z++) {
                //SDF.gridToWorld(x,y,z, posn.x, posn.y, posn.z);
                distance = EDT.getDistanceFromCell( x, y, z) - EDTnot.getDistanceFromCell( x, y, z);
                SDF.setCostOfCell( x,y,z, obstacleCost(distance) );
                //                 SDF.setCell(x,y,z,distance);

            }
        }
    }
    
    cout << "size " << SDFg.sx << "m x" << SDFg.sy << "m x" <<  SDFg.sz << "m \n";
    cout << "cells " << SDFg.nx << "x" << SDFg.ny << "x" <<  SDFg.nz << "\n";
    cout << "origin (" << SDFg.ox << "m," << SDFg.oy << "m, " <<  SDFg.oz << "m) \n";
    cout << "resolution " << SDFg.res << "\n";

#if 0
    double xlow = SDFg.ox+2*(SDFg.res), xhigh = SDFg.ox+SDFg.sx-2*SDFg.res;
    double ylow = SDFg.oy+2*(SDFg.res), yhigh = SDFg.oy+SDFg.sy-2*SDFg.res;
    double zlow = SDFg.oz+2*(SDFg.res), zhigh = SDFg.oz+SDFg.sx-2*SDFg.res;
    cout << "("<< xlow << " " << xhigh << "), << (" << ylow << " " << yhigh << "), << (" << zlow << " " << zhigh << ")\n" ;
    for (double z=zlow; z< zhigh; z+= 2*SDFg.res) {
        cout << "z: " << z << " \n y:     ";
        for (double y=ylow; y< yhigh; y+= 2*SDFg.res) {
            cout << y << " " ;
        }
        for (double x=xlow; x< xhigh; x+= 2*SDFg.res) {
            cout << "\n x:(" << x << ") ";
            for (double y=ylow; y< yhigh; y+= 2*SDFg.res)
            {
                cout << setiosflags(ios::fixed) << setprecision(2) << SDF.getCost(x,y,z) << " ";
                //                 cout << setiosflags(ios::fixed) << setprecision(2) << SDF.getCostFromCell(x,y,z) << " ";
            }
            cout << "\n";
        }
        cout << "\n";
    }
    cout << "printed cost field values\n";
#endif

#if 0
    double xlow = SDFg.ox+2*(SDFg.res), xhigh = SDFg.ox+SDFg.sx-2*SDFg.res;
    double ylow = SDFg.oy+2*(SDFg.res), yhigh = SDFg.oy+SDFg.sy-2*SDFg.res;
    double zlow = SDFg.oz+2*(SDFg.res), zhigh = SDFg.oz+SDFg.sx-2*SDFg.res;
    double grad[3], cost; //posn[3];
    cout << "("<< xlow << " " << xhigh << "), << (" << ylow << " " << yhigh << "), << (" << zlow << " " << zhigh << ")\n" ;
    for (double z=zlow; z< zhigh; z+= 2*SDFg.res) {
        cout << "z: " << z << " \n y:     ";
        for (double y=ylow; y< yhigh; y+= 2*SDFg.res) {
            cout << y << " " ;
        }
        for (double x=xlow; x< xhigh; x+= 2*SDFg.res) {
            cout << "\n x:(" << x << ") ";
            for (double y=ylow; y< yhigh; y+= 2*SDFg.res)
            {
                cout << setiosflags(ios::fixed) << setprecision(2) << SDF.getCost(x,y,z) << " ";
                //                 cout << setiosflags(ios::fixed) << setprecision(2) << SDF.getCostFromCell(x,y,z) << " ";
            }
            cout << "\n";
            for (double y=ylow; y< yhigh; y+= 2*SDFg.res)
            {
                //                 SDF.gridToWorld(x,y,z, posn[0], posn[1], posn[2]);
                cost = SDF.getCostGradient(x,y,z, grad[0], grad[1], grad[2]);
                
                cout << " (";
                for (int i = 0; i<3; i++){
                    cout << setiosflags(ios::fixed) << setprecision(2) << grad[i] << ",";
                }
                cout << ") ";
                //                 cout << setiosflags(ios::fixed) << setprecision(2) << SDF.getCostFromCell(x,y,z) << " ";
            }
            cout << "\n";
        }
        cout << "\n";
    }
    cout << "printed cost field values\n";
#endif

    return(SDF);
}
*/

double obstacleCost( double distance) // smooth obstacle cost -- 0 for positive distance for 
{
    if (distance > PADDING) {
        return 0.;
    } else if (distance < 0.) {
        return (-distance + 0.5*PADDING);
    } else { // between 0 and EPSILON
        return (1./(2*PADDING)*(distance - PADDING)*(distance-PADDING) ); // 1/(2e)*(dist-e)^2
    }
}

//! Generates the collision point for a given link
//! Stores the segment number (the id of the joint for planning)
//! Also store the parent joints of the link
std::vector<CollisionPoint> getLinksCollisionPoints( OpenRAVE::KinBody::JointPtr j, Vector p1, Vector p2, int segment_number, const std::vector<int>& parent_joints, double radius )
{
    std::vector<CollisionPoint> collision_points;

    double collision_clearance_default( radius );

    // double radius = 0.20; // bc->getRadius();

    double length = std::sqrt( (p1-p2).lengthsqr2() ); // bc->getLength();

    double spacing = radius / 2.0;
    int num_points = ceil( length / spacing ) + 1;
    spacing = length / ( num_points - 1.0 );

    cout << "segment number : " << segment_number << endl;

    for (int i=0; i<num_points; ++i)
    {
        Vector p = p1 + ((double)i/(double)num_points)*( p2 - p1 );
        collision_points.push_back( CollisionPoint( j, parent_joints, radius, collision_clearance_default, segment_number, p) );
    }

    return collision_points;
}

//! Generates the collision points for a given robot
//! iterates through the active joints
//! the radius for all the collision points is given as input
//! TODO compute the radii from the KinBody size
std::vector<CollisionPoint> createCollionPointsForRobot( OpenRAVE::RobotBasePtr robot, const std::vector<float>& radii, bool use_second_joint )
{
    std::vector<CollisionPoint> collision_points;

    const std::vector< int >& active_dofs = robot->GetActiveDOFIndices();
    std::vector< OpenRAVE::KinBody::JointPtr > joints;

    for( size_t i=0; i<active_dofs.size(); i++ )
    {
        OpenRAVE::KinBody::JointPtr j = robot->GetJointFromDOFIndex( active_dofs[i] );

        if( find( joints.begin(), joints.end(), j ) == joints.end() )
        {
            joints.push_back( j );
        }
    }

    for( size_t i=0; i<joints.size()-1; i++ )
    {
        OpenRAVE::KinBody::JointPtr j1 = joints[i];
        OpenRAVE::KinBody::JointPtr j2 = joints[i+1];

        cout << "j1.GetName() : " << j1->GetName() << endl;
        cout << "j2.GetName() : " << j2->GetName() << endl;

        Vector p1 = j1->GetHierarchyParentLink()->GetTransform().inverse() * j1->GetAnchor();
        Vector p2 = j2->GetHierarchyParentLink()->GetTransform().inverse() * j2->GetAnchor();

        // Make sure the array of radii is big enough
        double radius( i >= radii.size() ? radii.back() : radii[i] );

        // Get the firs or second joint (2nd in the case of the Puck Robot)
        OpenRAVE::KinBody::JointPtr joint_attached( use_second_joint ? j2 : j1 );

        // Get links and parent joints
        std::vector<int> parent_joints;
        std::vector<CollisionPoint> new_points = getLinksCollisionPoints( joint_attached, p1, p2, i, parent_joints, radius );
        collision_points.insert( collision_points.end(), new_points.begin(), new_points.end() );
    }

    return collision_points;
}
