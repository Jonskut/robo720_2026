#ifndef ROBO720_2026_PROJECT__PROJECT_CONTROLLER_HPP_
#define ROBO720_2026_PROJECT__PROJECT_CONTROLLER_HPP_

#include "franka_kdl/robot_constants.hpp"
#include "franka_kdl/solver.hpp"

#include <atomic>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <custom_msgs/msg/obstacle_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
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

#include <joint_limits/joint_limits.hpp>
#include <joint_limits/joint_limits_urdf.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace project {

class ProjectController : public controller_interface::ControllerInterface {
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

private:
  // subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr marker_pose_subscriber_;
  rclcpp::Subscription<custom_msgs::msg::ObstacleArray>::SharedPtr init_obstacle_subscriber_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_obj_pose_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_torque_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr fake_wrench_subscriber_;
  // realtime buffers
  realtime_tools::RealtimeBuffer<std::shared_ptr<KDL::Frame>> rt_goal_pose_buffer_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<KDL::Frame>> rt_marker_pose_buffer_;
  realtime_tools::RealtimeBuffer<std::vector<custom_msgs::msg::Obstacle>> rt_obstacles_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<Vector6d>> rt_force_torque_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<Vector6d>> rt_fake_wrench_;
  // publishers
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // parameters
  std::string robot_description_;
  std::vector<std::string> command_interface_types_;
  std::vector<std::string> state_interface_types_;
  Vector7d position_interface_vals_;
  Vector7d velocity_interface_vals_;
  std::vector<std::string> joint_names_;
  std::string root_link_;
  std::string tip_link_;
  std::string optical_link_;
  std::vector<double> p_gains_;
  std::vector<double> d_gains_;
  std::vector<double> wait_offset_;
  std::vector<double> wait_orientation_;
  double threshold_ = 0.12;
  double k_ = 2.0;
  double wn_ = 3.0;
  double zeta_ = 0.8;
  double md_scale_ = 0.2;

  // KDL / solver
  KDL::Tree tree_;
  KDL::Chain chain_;
  std::unique_ptr<Solver> solver_;

  // Camera chain (base -> optical link) for visual servoing
  KDL::Chain base_camera_chain_;
  std::unique_ptr<Solver> camera_solver_;
  KDL::Frame T_base_cam_;
  KDL::JntArray q_camera_;

  // state
  KDL::JntArray q_kdl_;
  KDL::JntArray q_dot_kdl_;
  KDL::Frame x_kdl_;       // current EE pose
  KDL::Frame xd_kdl_;      // goal EE pose
  KDL::JntArray q_desired_;
  KDL::JntArray gravity_;
  Vector7d tau_att_;
  Vector7d tau_rep_;
  Vector7d tau_cmd_;
  KDL::JntArray joint_min_;
  KDL::JntArray joint_max_;

  // Obstacle data
  std::unordered_map<std::string, custom_msgs::msg::Obstacle> obstacle_map_;

  // Visual servoing
  std::atomic<bool> marker_detected_{false};
  KDL::Frame T_cam_marker_;

  // Impedance / dynamics
  Eigen::Matrix<double, NUM_JOINTS, NUM_JOINTS> M_;
  Vector7d G_;
  Vector7d C_;
  Eigen::Matrix<double, NUM_TASK, NUM_JOINTS> J_;
  Eigen::Matrix<double, NUM_JOINTS, NUM_TASK> J_pinv_;
  Eigen::Matrix<double, NUM_TASK, NUM_JOINTS> J_dot_;
  Vector7d q_;
  Vector7d q_dot_;
  Vector6d h_e_;           // external measured forces/torques
  Vector6d h_e_old_;       // low-pass filter
  Vector7d tau_;           // torque command
  Vector7d tau_pos_;
  Vector7d tau_null_;
  Vector7d e_;             // joint position error
  KDL::JntSpaceInertiaMatrix M_kdl_;
  KDL::JntArray G_kdl_;
  KDL::JntArray C_kdl_;
  KDL::Jacobian J_kdl_;
  KDL::Jacobian J_dot_kdl_;
  Eigen::Matrix<double, 6, 6> Kp_;
  Eigen::Matrix<double, 6, 6> Kd_;
  Eigen::Matrix<double, 6, 6> Md_;
  Vector6d xd_ddot_ = Vector6d::Zero();
  Vector6d x_dot_tilde_;
  Vector6d x_dot_;
  Vector6d x_tilde_;
  double elapsed_time_{0.0};

  // Force-sensing flags
  std::atomic<bool> use_force_measurement_{true};
  std::atomic<bool> use_fake_wrench_{false};
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  // helpers
  double repulsiveAPF(double distance, double distance_prime);
  void publish_markers();
  void publish_goal_tf();
  void updateJointStates();
  void kdl_chain_error_print();
  void kdl_chain_success_print();

  // callbacks
  void goal_pose_callback(const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg);
  void marker_pose_callback(const std::shared_ptr<std_msgs::msg::Float64MultiArray> msg);
  void init_obstacle_callback(const std::shared_ptr<custom_msgs::msg::ObstacleArray> msg);
  void object_pose_callback(const tf2_msgs::msg::TFMessage& msg);
  void force_torque_callback(const std::shared_ptr<geometry_msgs::msg::WrenchStamped> msg);
  void fake_wrench_callback(const std::shared_ptr<geometry_msgs::msg::WrenchStamped> msg);
};

}  // namespace project

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(project::ProjectController,
                       controller_interface::ControllerInterface)

#endif  // ROBO720_2026_PROJECT__PROJECT_CONTROLLER_HPP_
