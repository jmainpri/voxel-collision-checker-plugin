#ifndef COLLISION_POINT_HPP
#define COLLISION_POINT_HPP

#include <vector>
#include <algorithm>

#include <rave/rave.h>

// #include "planner/TrajectoryOptim/Chomp/chompUtils.hpp"

namespace distance_field
{

class BoundingCylinder
{
public:

    BoundingCylinder(const OpenRAVE::Vector& p1, const OpenRAVE::Vector& p2, double radius) :
        m_p1(p1), m_p2(p2), m_radius(radius)
    {}

    BoundingCylinder(const OpenRAVE::Vector& vect1, const OpenRAVE::Vector& vect2,
                     const OpenRAVE::Vector& vect5, const OpenRAVE::Vector& vect6);

    double getLength() { return std::sqrt( ( m_p1 - m_p2 ).lengthsqr3() ); }
    double getRadius() { return m_radius; }

    OpenRAVE::Vector& getPoint1() { return m_p1; }
    OpenRAVE::Vector& getPoint2() { return m_p2; }

    void draw( std::vector< boost::shared_ptr<void> >& graphptr, const OpenRAVE::TransformMatrix& T );

private:
    OpenRAVE::Vector m_p1;
    OpenRAVE::Vector m_p2;
    double m_radius;
};

class CollisionPoint
{
public:

    CollisionPoint(OpenRAVE::KinBody::JointPtr j, const std::vector<int>& parent_joints, double radius, double clearance, int segment_number, const OpenRAVE::Vector& position);

    CollisionPoint(const CollisionPoint &point, const std::vector<int>& parent_joints);

    virtual ~CollisionPoint();

    bool isParentJoint(int joint) const;
    double getRadius() const;
    double getVolume() const;
    double getClearance() const;
    double getInvClearance() const;
    int getSegmentNumber() const;

    const OpenRAVE::Vector& getPosition() const;

    const std::vector<int>& getParentJoints() const
    {
        return m_parent_joints;
    }

    void getTransformMatrixedPosition(std::vector<OpenRAVE::TransformMatrix>& segment_frames, OpenRAVE::Vector& position) const;
    void getTransformMatrixedPosition(std::vector<std::vector<double> >& segment_frames, OpenRAVE::Vector& position) const;

    /*
    void getJacobian(std::vector<OpenRAVE::Vector>& joint_pos,
                     std::vector<OpenRAVE::Vector>& joint_axis,
                     OpenRAVE::Vector& collision_point_pos,
                     Eigen::MatrixXd& jacobian,
                     const std::vector<int>& group_joint_to_move3d_joint_index) const;
                     */

    void draw( std::vector< boost::shared_ptr<void> >& graphptr, OpenRAVE::EnvironmentBasePtr penv, bool yellow=true) const;

    bool m_is_colliding;                  /**< Collision point in collision */

private:
    OpenRAVE::KinBody::JointPtr m_joint; // Joint to draw the collision point

    std::vector<int> m_parent_joints;      /**< Which joints can influence the motion of this point */
    double m_radius;                       /**< Radius of the sphere */
    double m_volume;                       /**< Volume of the sphere */
    double m_clearance;                    /**< Extra clearance required while optimizing */
    double m_inv_clearance;                /**< 1/clearance_ pre-computed */
    int m_segment_number;                  /**< Which segment does this point belong to */

    OpenRAVE::Vector m_position;            /**< Vector of this point in the frame of the above segment */

    inline void stdVectorToEigenTransformMatrix(const std::vector<double>& stl, OpenRAVE::TransformMatrix& T) const
    {
        for (int j=0; j<4; j++)
        {
            for (int i=0; i<3; i++)
            {
                T.rot(i,j) = stl[i*4+j];
            }
        }

        T.trans[0] = 0;
        T.trans[1] = 0;
        T.trans[2] = 0;

        //cout << "Transfo : " << endl << T.matrix() << endl;
    }
};

inline bool CollisionPoint::isParentJoint(int joint) const
{
    return(find(m_parent_joints.begin(),
                m_parent_joints.end(), joint) != m_parent_joints.end());
}

inline double CollisionPoint::getRadius() const
{
    return m_radius;
}

inline double CollisionPoint::getVolume() const
{
    return m_volume;
}

inline double CollisionPoint::getClearance() const
{
    return m_clearance;
}

inline double CollisionPoint::getInvClearance() const
{
    return m_inv_clearance;
}

inline int CollisionPoint::getSegmentNumber() const
{
    return m_segment_number;
}

inline const OpenRAVE::Vector& CollisionPoint::getPosition() const
{
    return m_position;
}

inline void CollisionPoint::getTransformMatrixedPosition(std::vector<OpenRAVE::TransformMatrix>& segment_frames,
                                                   OpenRAVE::Vector& position) const
{
    position = segment_frames[m_segment_number] * m_position;
}

inline void CollisionPoint::getTransformMatrixedPosition(std::vector<std::vector<double> >& segment_frames,
                                                   OpenRAVE::Vector& position) const
{
    OpenRAVE::TransformMatrix T;
    stdVectorToEigenTransformMatrix( segment_frames[m_segment_number], T );
    position = T*m_position;
}

}

#endif // COLLISION_POINT_HPP
