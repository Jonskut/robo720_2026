#ifndef ROBO720_2026_FRANKA_KDL__SOLVER_HPP_
#define ROBO720_2026_FRANKA_KDL__SOLVER_HPP_

#include "franka_kdl/robot_constants.hpp"

#include <string>
#include <memory>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>


/**
 * Class to compute the inverse position kinematics from Cartesian space
 * to joint space.
 */
class Solver {
    public:
        Solver(const KDL::Chain& chain);

        int computeIK(const KDL::JntArray& q_init, const KDL::Frame& target_pose, KDL::JntArray& result);

        int computeFK(const KDL::JntArray& q, KDL::Vector& pos, const int& seg_nr=-1);
        int computeFK(const KDL::JntArray& q, KDL::Frame& pose, const int& seg_nr=-1);

        int computeIKvel(const KDL::JntArray& q_init, const KDL::Twist& vel, KDL::JntArray& q_dot);

        void kdl_debug_print(const KDL::JntArray& jnt_array);

    private:
        KDL::Chain chain_;
        std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
        std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
        std::unique_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
        std::unique_ptr<KDL::ChainDynParam> dyn_param_solver_;
        std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
        std::unique_ptr<KDL::ChainJntToJacDotSolver> jac_dot_solver_;

        KDL::Vector g_; // gravity vector
};

#endif  // ROBO720_2026_FRANKA_KDL__SOLVER_HPP_