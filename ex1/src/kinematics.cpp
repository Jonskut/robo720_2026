#include "kinematics.hpp"

Kinematics::Kinematics(const float &L1, const float &L2, const float &L3)
    : L1_(L1), L2_(L2), L3_(L3)
{
}

Eigen::Vector2f Kinematics::compute_ee_pos(const float &q1, const float &q2, const float& q3)
{
    /**
     * TODO: Compute end effector position using trigonometrics
     */
    float x;
    float y;
    x = L1_ * cos(q1) + L2_ * cos(q1 + q2) + L3_ * cos(q1 + q2 + q3);
    y = L1_ * sin(q1) + L2_ * sin(q1 + q2) + L3_ * sin(q1 + q2 + q3);
    return Eigen::Vector2f(x,y);
}

Eigen::Matrix3f Kinematics::compute_fk_eigen(const float &q1, const float &q2, const float& q3)
{
    // Construct transformation matrices from each joint to next one
    Eigen::Matrix3f T_base_1, T_1_2, T_2_3, T_3_ee;
    
    T_base_1<<cos(q1), -sin(q1), L1_*cos(q1),
               sin(q1), cos(q1), L1_*sin(q1),
               0, 0, 1;
    T_1_2<<cos(q2), -sin(q2), L2_*cos(q2),
            sin(q2), cos(q2), L2_*sin(q2),
            0, 0, 1;
    T_2_3<<cos(q3), -sin(q3), L3_*cos(q3),
            sin(q3), cos(q3), L3_*sin(q3),
            0, 0, 1;
    T_3_ee = Eigen::Matrix3f::Identity();
    /**
     * TODO: Populate the transformation matrices from joint to joint,
     * then chain them to get the transformation from 'base' to 'end effector'.
     */

    Eigen::Matrix3f T_base_ee;
    T_base_ee = T_base_1 * T_1_2 * T_2_3 * T_3_ee;
    return T_base_ee;
}

void Kinematics::construct_kdl_chain()
{
    /**
     * TODO: Construct KDL Chain object
     */
    chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(L1_, 0.0, 0.0))));
    chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(L2_, 0.0, 0.0))));
    chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(L3_, 0.0, 0.0))));
}

KDL::Frame Kinematics::compute_fk_kdl(const float &q1, const float &q2, const float& q3)
{
    /**
     * TODO: Construct forward kinematic solver object,
     * and solve end effector pose given the joint values
     */

        KDL::ChainFkSolverPos_recursive fk_solver(chain_);

        KDL::JntArray q(3);
        q(0) = q1;
        q(1) = q2;
        q(2) = q3;
        KDL::Frame end_effector_pose;
        fk_solver.JntToCart(q, end_effector_pose);
    return end_effector_pose;
    /**return KDL::Frame(ee_frame); */
}

KDL::JntArray Kinematics::compute_ik_kdl(const float &q1_init, const float &q2_init, const float& q3_init, const KDL::Frame &target_pose)
{
    /**
     * TODO: Construct inverse kinematics solver object,
     * and solve joint positions given the desired pose
     * and initial joint values.
     */

    KDL::ChainIkSolverPos_LMA ik_solver(chain_);

    KDL::JntArray q_init(3);
    KDL::JntArray q_out(3);

    q_init(0) = q1_init;
    q_init(1) = q2_init;
    q_init(2) = q3_init;

    ik_solver.CartToJnt(q_init, target_pose, q_out);

    return q_out;
    /**return KDL::JntArray(q_out); */
}

KDL::Jacobian Kinematics::compute_jac_kdl(const float &q1, const float &q2, const float& q3,
                                          const int& segment_n)
{
    /**
     * TODO: Construct Jacobian solver object, and
     * solve the Jacobian given the joint values.
     */
    
    KDL::ChainJntToJacSolver jac_solver(chain_);

    KDL::JntArray q(3);
    q(0) = q1;
    q(1) = q2;
    q(2) = q3;

    KDL::Jacobian jacobian(3);

    jac_solver.JntToJac(q, jacobian, segment_n);

    return jacobian;


    /**return KDL::Jacobian();*/
}
