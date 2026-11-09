#ifndef ROBO720_2026_EX7__IMPEDANCE_CONTROLLER_HPP_
#define ROBO720_2026_EX7__IMPEDANCE_CONTROLLER_HPP_

#include "franka_kdl/robot_constants.hpp"
#include "franka_kdl/solver.hpp"

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <urdf/model.h>

#include <Eigen/Dense>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace ex7 {

class ImpedanceController : public controller_interface::ControllerInterface {
    public:
        // Controller interface
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;
        controller_interface::return_type update(
            const rclcpp::Time& time, const rclcpp::Duration& period) override;
        CallbackReturn on_init() override;
        CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

    private:
        // Non-RT (realtime) subscriber
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_subscriber_;
        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_torque_subscriber_;
        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr fake_wrench_subscriber_;
        
        // Buffer for RT update() loop (store a shared_ptr instead of the message itself -> less copying)
        realtime_tools::RealtimeBuffer<std::shared_ptr<KDL::Frame>> rt_goal_pose_buffer_;
        realtime_tools::RealtimeBuffer<std::shared_ptr<Vector6d>> rt_force_torque_;
        realtime_tools::RealtimeBuffer<std::shared_ptr<Vector6d>> rt_fake_wrench_;

        std::string robot_description_;
        std::vector<std::string> command_interface_types_;
        std::vector<std::string> state_interface_types_;

        Vector7d position_interface_vals_;
        Vector7d velocity_interface_vals_;
        
        std::vector<std::string> joint_names_;
        std::string root_link_;
        std::string tip_link_;

        KDL::Tree tree_;
        KDL::Chain chain_;
        std::unique_ptr<Solver> solver_;

        // Eigen variables
        Eigen::Matrix<double, NUM_JOINTS, NUM_JOINTS> M_;
        Vector7d G_;
        Vector7d C_;
        Eigen::Matrix<double, NUM_TASK, NUM_JOINTS> J_;
        Eigen::Matrix<double, NUM_JOINTS, NUM_TASK> J_pinv_;
        Eigen::Matrix<double, NUM_TASK, NUM_JOINTS> J_dot_;
        Vector7d q_;
        Vector7d q_dot_;
        Vector6d h_e_;  // external measured forces and torques
        Vector6d h_e_old_;  // for low-pass filtering
        Vector7d tau_; // torque command
        Vector7d tau_pos_;
        Vector7d tau_null_;

        // KDL variables
        KDL::JntSpaceInertiaMatrix M_kdl_;
        KDL::JntArray G_kdl_;
        KDL::JntArray C_kdl_;
        KDL::Jacobian J_kdl_;
        KDL::Jacobian J_dot_kdl_;
        KDL::JntArray q_kdl_;
        KDL::JntArray q_dot_kdl_;
        KDL::Frame x_kdl_;
        KDL::Frame xd_kdl_;
        
        // ex7 added variables
        KDL::Rotation R_bs_;
        Eigen::Matrix<double, 6, 6> W_;
        double Kd_;
        double Kp_;
        Vector7d u_;
        Vector6d xd_ddot_ = Vector6d::Zero();
        Vector6d x_dot_tilde_;
        Vector6d x_dot_;
        Vector6d x_tilde_;

        // ex7 desired params
        Eigen::Matrix<double, 6, 6> Md_;
        double wn_ = 10;
        double zeta_ = 1

        double elapsed_time_{0.0};

        // For writing the parameter on non-RT thread and reading it in RT update() loop
        std::atomic<bool> use_force_measurement_{true};
        std::atomic<bool> use_fake_wrench_{false};
        rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

        // Subscriber callbacks
        void goal_pose_callback(const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg);
        void force_torque_callback(const std::shared_ptr<geometry_msgs::msg::WrenchStamped> msg);
        void fake_wrench_callback(const std::shared_ptr<geometry_msgs::msg::WrenchStamped> msg);

        // Read joint states from the hardware state interface
        void updateJointStates();

        // Debug print functions to reduce clutter in the controller code
        void kdl_chain_error_print();
        void kdl_chain_success_print();
};

}  // namespace ex7

#endif  // ROBO720_2026_EX7__IMPEDANCE_CONTROLLER_HPP_