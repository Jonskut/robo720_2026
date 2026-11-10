#ifndef ROBO720_2026_EX6__POTENTIAL_FIELD_CONTROLLER_HPP_
#define ROBO720_2026_EX6__POTENTIAL_FIELD_CONTROLLER_HPP_

#include "franka_kdl/robot_constants.hpp"
#include "franka_kdl/solver.hpp"

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <custom_msgs/msg/obstacle_array.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <urdf/model.h>

#include <Eigen/Dense>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>

#include <cmath>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace ex6 {

class PotentialFieldController : public controller_interface::ControllerInterface {
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
        // Subscriber for obstacle initial positions and shape
        rclcpp::Subscription<custom_msgs::msg::ObstacleArray>::SharedPtr init_obstacle_subscriber_;
        // Subscriber for obstacle positions
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_obj_pose_;
        
        // Buffer for RT update() loop (store a shared_ptr instead of the message itself -> less copying)
        realtime_tools::RealtimeBuffer<std::shared_ptr<KDL::Frame>> rt_goal_pose_buffer_;
        realtime_tools::RealtimeBuffer<std::vector<custom_msgs::msg::Obstacle>> rt_obstacles_;

        // Marker publisher for RViz visualization
        rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

        std::string robot_description_;
        std::vector<std::string> command_interface_types_;
        std::vector<std::string> state_interface_types_;

        Vector7d position_interface_vals_;
        Vector7d velocity_interface_vals_;

        // For storing the obstacle data
        std::unordered_map<std::string, custom_msgs::msg::Obstacle> obstacle_map_;
        
        std::vector<std::string> joint_names_;
        std::string root_link_;
        std::string tip_link_;

        KDL::Tree tree_;
        KDL::Chain chain_;
        std::unique_ptr<Solver> solver_;

        // KDL variables
        KDL::JntArray q_kdl_;
        KDL::JntArray q_dot_kdl_;
        KDL::Frame xd_kdl_;

        // Own variables
        KDL::JntArray G_compensation_kdl_;
        KDL::JntArray q_goal_kdl_;
        double d_;
        double dprime_;
        Vector7d dprime_vec_;
        Vector7d tau_att_;
        Vector7d tau_rep_;
        Vector7d tau_;
        Vector7d F_rep_;
        const double Kp_ = 25; // Follows from the K_d tuning
        const double Kd_ = 2*0.9*5; // Damping ratio = 0.9, desired_bandwith = 5 rad/s
        Vector7d q_min_ = Vector7d{
            -2.8973, -1.7628, -2*M_PI, -2*M_PI, -2*M_PI, -2*M_PI, -2*M_PI};
        Vector7d q_max_ = Vector7d{
            2.8973, 1.7628, 2*M_PI, 2*M_PI, 2*M_PI, 2*M_PI, 2*M_PI};
        Vector7d G_compensation;
        Vector7d q;
        Vector7d q_dot;
        Vector7d q_d;


        double elapsed_time_{0.0};
        
        // Own functions
        double RepulsiveForce(double d, double ddot, double threshold_distance, double k_RPF);
        // Subscriber callbacks
        void goal_pose_callback(const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg);
        void init_obstacle_callback(const custom_msgs::msg::ObstacleArray msg);
        void object_pose_callback(const tf2_msgs::msg::TFMessage& msg);

        // Publishers
        void publish_markers();

        // Read joint states from the hardware state interface
        void updateJointStates();

        // Debug print functions to reduce clutter in the controller code
        void kdl_chain_error_print();
        void kdl_chain_success_print();
};

}  // namespace ex6

#endif  // ROBO720_2026_EX6__POTENTIAL_FIELD_CONTROLLER_HPP_