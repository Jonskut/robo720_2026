#include "kinematics.hpp"

using namespace KDL;

Kinematics::Kinematics(const float &L1, const float &L2, const float &L3)
    : L1_(L1), L2_(L2), L3_(L3)
{
}

Eigen::Vector2f Kinematics::compute_ee_pos(const float &q1, const float &q2, const float& q3)
{
    float x = L1_ * cos(q1) + L2_ * cos(q1 + q2) + L3_ * cos(q1 + q2 + q3);
    float y = L1_ * sin(q1) + L2_ * sin(q1 + q2) + L3_ * sin(q1 + q2 + q3);

    return Eigen::Vector2f(x, y);
}

Eigen::Matrix3f Kinematics::compute_fk_eigen(const float &q1, const float &q2, const float& q3)
{
    // Construct transformation matrices from each joint to next one
    Eigen::Matrix3f T_base_1, T_1_2, T_2_3, T_3_ee;

    /**
     * TODO: Populate the transformation matrices from joint to joint,
     * then chain them to get the transformation from 'base' to 'end effector'.
     */
    T_base_1 << cos(q1), -sin(q1), L1_*cos(q1),
                sin(q1),  cos(q1), L1_*sin(q1),
                0,        0,       1;

    T_1_2 << cos(q2), -sin(q2), L2_ * cos(q2),
    sin(q2),  cos(q2), L2_ * sin(q2),
    0,        0,       1;
   
    T_2_3 << cos(q3), -sin(q3), L3_ * cos(q3),
            sin(q3),  cos(q3), L3_ * sin(q3),
            0,        0,       1;
   
    Eigen::Matrix3f T_base_ee = T_base_1 * T_1_2 * T_2_3;

    return T_base_ee;
}

void Kinematics::construct_kdl_chain()
{
    /**
     * TODO: Construct KDL Chain object
     */
    KDL::Chain chain;
    chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(L1_,0.0,0.0))));
    chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(L2_,0.0,0.0))));
    chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(L3_,0.0,0.0))));
}

KDL::Frame Kinematics::compute_fk_kdl(const float &q1, const float &q2, const float& q3)
{
    /**
     * TODO: Construct forward kinematic solver object,
     * and solve end effector pose given the joint values
     */
    KDL::Chain chain;
    chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(L1_,0.0,0.0))));
    chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(L2_,0.0,0.0))));
    chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(L3_,0.0,0.0))));

    ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);

    // Create joint array
    unsigned int nj = chain.getNrOfJoints();
    KDL::JntArray jointpositions = JntArray(nj);
    jointpositions(0) = q1;
    jointpositions(1) = q2;
    jointpositions(2) = q3;
 
    // Create the frame that will contain the results
    KDL::Frame cartpos;    
 
    // Calculate forward position kinematics
    bool kinematics_status;
    kinematics_status = fksolver.JntToCart(jointpositions,cartpos);

    return cartpos;
}

KDL::JntArray Kinematics::compute_ik_kdl(const float &q1_init, const float &q2_init, const float& q3_init, const KDL::Frame &target_pose)
{
    /**
     * TODO: Construct inverse kinematics solver object,
     * and solve joint positions given the desired pose
     * and initial joint values.
     */

    KDL::Chain chain;
    chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(L1_,0.0,0.0))));
    chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(L2_,0.0,0.0))));
    chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(L3_,0.0,0.0))));

    //Creation of the solvers:
    ChainFkSolverPos_recursive fksolver1(chain);//Forward position solver
    ChainIkSolverPos_LMA iksolver1v(chain, 1e-6, 100, 1e-6); //Inverse position solver
      
    unsigned int nj = chain.getNrOfJoints();
    JntArray q(nj);
    KDL::JntArray q_init = JntArray(nj);
    q_init(0) = q1_init;
    q_init(1) = q2_init;
    q_init(2) = q3_init;

    int ret = iksolver1v.CartToJnt(q_init, target_pose, q);
    return q;
}

KDL::Jacobian Kinematics::compute_jac_kdl(const float &q1, const float &q2, const float& q3,
                                          const int& segment_n)
{
    /**
     * TODO: Construct Jacobian solver object, and
     * solve the Jacobian given the joint values.
     */
    KDL::Chain chain;
    chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(L1_,0.0,0.0))));
    chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(L2_,0.0,0.0))));
    chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(L3_,0.0,0.0))));

    KDL::ChainJntToJacSolver jac_solver(chain);

    KDL::JntArray joint_positions(chain.getNrOfJoints());
    joint_positions(0) = q1;
    joint_positions(1) = q2;
    joint_positions(2) = q3;

    KDL::Jacobian jacobian(chain.getNrOfJoints());
    int result = jac_solver.JntToJac(joint_positions, jacobian, segment_n);

    return jacobian;
}
