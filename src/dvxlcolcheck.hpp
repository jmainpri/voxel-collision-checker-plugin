#ifndef DVXLCOLCHECK_H
#define DVXLCOLCHECK_H

#include "stdafx.h"
#include "libDVXL.h"

//this collision checker currently fills the numWithinTol variable of the CollisionReport, nothing more
class DVXLCollisionChecker : public OpenRAVE::CollisionCheckerBase
{
public:
    DVXLCollisionChecker(EnvironmentBasePtr penv, DVXL_GRID grid_in, Transform Tdg_in);
    DVXLCollisionChecker(EnvironmentBasePtr penv);
    ~DVXLCollisionChecker() {}

    /// Set basic collision options using the CollisionOptions enum
    virtual bool SetCollisionOptions(int collisionoptions) {return false;}
    virtual int GetCollisionOptions() const {return -1;}

    /// set and get various collision checker options
    /// \return true if command succeeded
    virtual bool SetCollisionOptions(std::ostream& sout, std::istream& sinput) {return true;}
    virtual void SetTolerance(dReal tolerance) {}


    /// called when environment sets this collision checker, checker assumes responsibility for KinBody::_pCollisionData
    /// checker should also gather all current bodies in the environment and put them in its collision space
    virtual bool InitEnvironment() {return true;}

    /// called when environment switches to a different collision checker engine
    /// has to clear/deallocate any memory associated with KinBody::_pCollisionData
    virtual void DestroyEnvironment() {}

    /// notified when a new body has been initialized in the environment
    virtual bool InitKinBody(KinBodyPtr pbody) {return true;}

    /// enables or disables a kinematic body from being considered in collisions
    /// \return true if operation succeeded
    virtual bool Enable(KinBodyConstPtr pbody, bool bEnable) {return true;}

    /// enables or disables a link from being considered in collisions
    /// \return true if operation succeeded
    virtual bool EnableLink(KinBody::LinkConstPtr pbody, bool bEnable) {return true;}

    virtual bool CheckCollision(KinBodyConstPtr pbody1, CollisionReportPtr report = CollisionReportPtr()){return false;}
    virtual bool CheckCollision(KinBodyConstPtr pbody1, KinBodyConstPtr pbody2, CollisionReportPtr report = CollisionReportPtr()){return false;}
    virtual bool CheckCollision(KinBody::LinkConstPtr plink, CollisionReportPtr report = CollisionReportPtr()){return false;}
    virtual bool CheckCollision(KinBody::LinkConstPtr plink1, KinBody::LinkConstPtr plink2, CollisionReportPtr report = CollisionReportPtr()){return false;}
    virtual bool CheckCollision(KinBody::LinkConstPtr plink, KinBodyConstPtr pbody, CollisionReportPtr report = CollisionReportPtr()){return false;}

    virtual bool CheckCollision(KinBody::LinkConstPtr plink, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<KinBody::LinkConstPtr>& vlinkexcluded, CollisionReportPtr report = CollisionReportPtr()){return false;}
    virtual bool CheckCollision(KinBodyConstPtr pbody, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<KinBody::LinkConstPtr>& vlinkexcluded, CollisionReportPtr report = CollisionReportPtr()){return false;}

    virtual bool CheckCollision(const RAY& ray, KinBody::LinkConstPtr plink, CollisionReportPtr report = CollisionReportPtr()){return false;}
    virtual bool CheckCollision(const RAY& ray, KinBodyConstPtr pbody, CollisionReportPtr report = CollisionReportPtr()){return false;}
    virtual bool CheckCollision(const RAY& ray, CollisionReportPtr report = CollisionReportPtr()){return false;}

    /// checks self collision only with the links of the passed in body
    virtual bool CheckSelfCollision(KinBodyConstPtr pbody, CollisionReportPtr report = CollisionReportPtr()){return false;}

    //voxel-specific check
    bool CheckCollision(KinBodyConstPtr pbody1, std::vector<std::vector<Vector> >& vvLinkPoints, CollisionReportPtr report = CollisionReportPtr());
    bool CheckCollision(KinBodyConstPtr pbody1, std::vector<std::vector<Vector> >& vvLinkPoints, std::vector<std::vector<bool> >& vvPointsColliding, CollisionReportPtr report = CollisionReportPtr());
    bool CheckCostFnCollision(KinBodyConstPtr pbody1, std::vector<std::vector<Vector> >& vvLinkPoints, std::vector<std::vector<float> >& vvPointsColliding, float* total_cost, CollisionReportPtr report);

    inline float ComputeCostFn(float sia, float sib, float dia, float dib)
    {
        if (dia == 0.0 && dib == 0.0)
        {
            return NAN;
        }
        else if (sia == 0.0 || sib == 0.0)
        {
            return 0.0;
        }
        else
        {
            float ca = (dia / (dia + dib)) * sia;
            float cb = (dib / (dia + dib)) * sib;
            return ca + cb;
        }
    }

    bool bInitialized_;
protected:
    DVXL_GRID grid_;
    Transform Tdg_;
    bool bDraw_;
    std::vector<RaveVector<float> > vplotpoints_;
    std::vector<float> vplotcolors_;
    std::vector<  boost::shared_ptr<void> > graphptrs;
};

#endif // DVXLCOLCHECK_H
