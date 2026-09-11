#include "franka_kdl/solver.hpp"
#include "franka_kdl/robot_constants.hpp"

#include <cmath>
#include <cstddef>
#include <iostream>

#include <kdl/frames.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>


Solver::Solver(const KDL::Chain& chain)
: chain_(chain), g_(0.0, 0.0, -9.8) { // Default gravity vector in Gazebo
    ik_solver_.reset(new KDL::ChainIkSolverPos_LMA(chain));
    fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(chain));
    ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(chain));
}

int Solver::computeIK(const KDL::JntArray& q_init, const KDL::Frame& target_pose, KDL::JntArray& result) {
    int ret = ik_solver_->CartToJnt(q_init, target_pose, result);

    return ret;
}

int Solver::computeFK(const KDL::JntArray& q, KDL::Vector& pos, const int& seg_nr) {
    KDL::Frame result;
    // store the end-effector position in result
    int ret = fk_solver_->JntToCart(q, result, seg_nr);
    pos = result.p;

    return ret;
}

int Solver::computeFK(const KDL::JntArray& q, KDL::Frame& pose, const int& seg_nr) {
    // store the end-effector pose
    int ret = fk_solver_->JntToCart(q, pose, seg_nr);

    return ret;
}

int Solver::computeIKvel(const KDL::JntArray& q_init, const KDL::Twist& vel, KDL::JntArray& q_dot) {
    int ret = ik_vel_solver_->CartToJnt(q_init, vel, q_dot);

    return ret;
}

void Solver::kdl_debug_print(const KDL::JntArray& jnt_array) {
    std::cout << "KDL JntArray" << std::endl << std::endl;

    for (unsigned int i = 0; i < jnt_array.rows(); i++) {
        std::cout << i << ": " << jnt_array(i) << std::endl;
    }
}