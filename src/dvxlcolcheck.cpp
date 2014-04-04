#include "stdafx.h"

#define DEFAULT_SIA 0.5
#define DEFAULT_DIA 0.5

DVXLCollisionChecker::DVXLCollisionChecker(EnvironmentBasePtr penv, DVXL_GRID grid_in, Transform Tdg_in): OpenRAVE::CollisionCheckerBase(penv), grid_(1.0,1.0,1.0,0.5,1.0,0.0,Transform())
{
    grid_ = grid_in;
    Tdg_ = Tdg_in;
    bDraw_ = false;
    bInitialized_ = true;
}

DVXLCollisionChecker::DVXLCollisionChecker(EnvironmentBasePtr penv): OpenRAVE::CollisionCheckerBase(penv), grid_(1.0,1.0,1.0,0.5,1.0,0.0,Transform())
{
    DVXL_GRID new_grid(1.0,1.0,1.0,0.5,1.0,0.0,Transform());
    grid_ = new_grid;
    bDraw_ = false;
    bInitialized_ = false;
}

bool DVXLCollisionChecker::CheckCollision(KinBodyConstPtr pbody1, std::vector<std::vector<Vector> >& vvLinkPoints, CollisionReportPtr report)
{
    std::vector<std::vector<bool> > vvPointsColliding;
    return CheckCollision(pbody1, vvLinkPoints, vvPointsColliding, report);

}

bool DVXLCollisionChecker::CheckCollision(KinBodyConstPtr pbody1, std::vector<std::vector<Vector> >& vvLinkPoints, std::vector<std::vector<bool> >& vvPointsColliding, CollisionReportPtr report)
{
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


    for(int i = 0; i <  pbody1->GetLinks().size(); i++)
    {
        Transform temptm = pbody1->GetLinks()[i]->GetTransform();
        vvPointsColliding[i].resize(vvLinkPoints[i].size());

        for(int j = 0; j < vvLinkPoints[i].size(); j++)
        {
            //RAVELOG_INFO("linktrans: %f %f %f\n",temptm.trans.x,temptm.trans.y,temptm.trans.z);
            Vector sample = temptm*vvLinkPoints[i][j];
            dvxl_t cell = grid_.grid_(sample.x, sample.y, sample.z);
            //RAVELOG_INFO("val: %f\n",val);
            if(cell.sensitivity > 0.0)
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
        graphptrs.push_back(figure);
    }

    if(report->numWithinTol)
        return true;
    else
        return false;

}

bool DVXLCollisionChecker::CheckCostFnCollision(KinBodyConstPtr pbody1, std::vector<std::vector<Vector> >& vvLinkPoints, std::vector<std::vector<float> >& vvPointsColliding, float* total_cost, CollisionReportPtr report)
{
    //Safety checks
    if(!bInitialized_)
    {
        RAVELOG_INFO("DVXLCollisionChecker ERROR: DVXLCollisionChecker is not initialized!\n");
        return false;
    }
    if(pbody1->GetLinks().size() != vvLinkPoints.size())
    {
        RAVELOG_INFO("DVXLCollisionChecker ERROR: Number of links in body (%d) and number of links in vvLinkPoints (%d) are different!\n",pbody1->GetLinks().size(),vvLinkPoints.size());
        return false;
    }

    //Init the output storage
    if( !report )
    {
        report.reset(new CollisionReport());
    }
    vvPointsColliding.resize(pbody1->GetLinks().size());
    report->numWithinTol = 0;

    float sum_cost = 0.0;
    //Do the actual work
    //Iterate through the links of the robot
    for(int i = 0; i < pbody1->GetLinks().size(); i++)
    {
        //Get the transform of the current link
        Transform temptm = pbody1->GetLinks()[i]->GetTransform();
        //Resize as necessary
        vvPointsColliding[i].resize(vvLinkPoints[i].size());

        for(int j = 0; j < vvLinkPoints[i].size(); j++)
        {
            //RAVELOG_INFO("linktrans: %f %f %f\n",temptm.trans.x,temptm.trans.y,temptm.trans.z);
            Vector sample = temptm*vvLinkPoints[i][j];
            dvxl_t cell = grid_.grid_(sample.x, sample.y, sample.z);
            float real_cost = this->ComputeCostFn(DEFAULT_SIA,cell.sensitivity,DEFAULT_DIA,cell.deformability);
            sum_cost += real_cost;
            if(isnan(real_cost))
            {
                vvPointsColliding[i][j] = real_cost;
            }
            else if (real_cost == 0.0)
            {
                vvPointsColliding[i][j] = real_cost;
            }
            else
            {
                report->numWithinTol++;
                vvPointsColliding[i][j] = real_cost;
            }
        }
    }
    *total_cost = sum_cost;
    if(sum_cost > 0.0)
    {
        return true;
    }
    else
    {
        return false;
    }
}
