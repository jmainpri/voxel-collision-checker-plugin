#include "collision_point.hpp"
#include <cmath>

using namespace distance_field;

CollisionPoint::CollisionPoint(
        OpenRAVE::KinBody::JointPtr j,
        const std::vector<int>& parent_joints,
        double radius,
        double clearance,
        int segment_number,
        const OpenRAVE::Vector& position) :
    m_joint(j),
    m_parent_joints(parent_joints),
    m_radius(radius),
    m_volume((4.0/3.0)*M_PI*m_radius*m_radius*m_radius),
    m_clearance(clearance),
    m_inv_clearance(1.0/m_clearance),
    m_segment_number(segment_number),
    m_position(position)
{

}

CollisionPoint::CollisionPoint(const CollisionPoint &point, const std::vector<int>& parent_joints):
    m_parent_joints(parent_joints),
    m_radius(point.m_radius),
    m_volume((4.0/3.0)*M_PI*m_radius*m_radius*m_radius),
    m_clearance(point.m_clearance),
    m_inv_clearance(1.0/m_clearance),
    m_segment_number(point.m_segment_number),
    m_position(point.m_position)
{

}

CollisionPoint::~CollisionPoint()
{

}

/*
void CollisionPoint::getJacobian(std::vector<OpenRAVE::Vector>& joint_pos,
                                 std::vector<OpenRAVE::Vector>& joint_axis,
                                 OpenRAVE::Vector& collision_point_pos,
                                 OpenRAVE::Matrix& jacobian,
                                 const std::vector<int>& group_joint_to_move3d_joint_index) const
{
    for(size_t joint = 0; joint < group_joint_to_move3d_joint_index.size(); joint++)
    {
        if(!isParentJoint(group_joint_to_move3d_joint_index[joint])) {
            // since the joint is not active, fill the jacobian column with zeros
            jacobian.col(joint).setZero();
        }
        else
        {
            //int kj = group_joint_to_move3d_joint_index[joint];
            jacobian.col(joint) = joint_axis[joint].cross( collision_point_pos - joint_pos[joint] );
            //jacobian.col(joint) = OpenRAVE::Vector::Zero();
        }
    }
}
*/

void CollisionPoint::draw( std::vector< boost::shared_ptr<void> >& graphptr, OpenRAVE::EnvironmentBasePtr penv, bool yellow ) const
{
    const OpenRAVE::TransformMatrix& T = m_joint->GetHierarchyParentLink()->GetTransform();
    OpenRAVE::Vector point = T*m_position;

    // yellow
    OpenRAVE::RaveVector<float> vcolors(1.0,1.0,0.0,0.1);

    std::vector<OpenRAVE::RaveVector<float> > vpoints;
    OpenRAVE::RaveVector<float> pnt( point[0], point[1], point[2] );
    vpoints.push_back( pnt );

    graphptr.push_back( penv->plot3( &vpoints[0].x, vpoints.size(), sizeof( vpoints[0]), m_radius, vcolors, 1 ) );
}

BoundingCylinder::BoundingCylinder(const OpenRAVE::Vector& vect1, const OpenRAVE::Vector& vect2,
                                   const OpenRAVE::Vector& vect5, const OpenRAVE::Vector& vect6)
{
    m_p1 = 0.5*(vect1+vect2);
    m_p2 = 0.5*(vect5+vect6);

    m_radius = 0.5*( std::sqrt( ( vect1 - vect2 ).lengthsqr3() ) );
}

void BoundingCylinder::draw( std::vector< boost::shared_ptr<void> >& graphptr, const OpenRAVE::TransformMatrix& T )
{
//    double p1[3], p2[3];

//    OpenRAVE::Vector p1Trans;
//    OpenRAVE::Vector p2Trans;

//    p1Trans = T*m_p1;
//    p2Trans = T*m_p2;

//    p1[0] = p1Trans[0];
//    p1[1] = p1Trans[1];
//    p1[2] = p1Trans[2];

//    p2[0] = p2Trans[0];
//    p2[1] = p2Trans[1];
//    p2[2] = p2Trans[2];

//    double colorvector[4];
//    colorvector[0] = 1.0;
//    colorvector[1] = 1.0;
//    colorvector[2] = 0.0;
//    colorvector[3] = 0.7;

//    glEnable(GL_BLEND);
//    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

//    glColor4dv(colorvector);
//    g3d_draw_cylinder(p1, p2, m_radius, 20 );

//    glDisable(GL_BLEND);
}


