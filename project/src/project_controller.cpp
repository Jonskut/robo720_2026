#include "project/project_controller.hpp"
#include "franka_kdl/robot_constants.hpp"
#include "franka_kdl/solver.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <exception>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <controller_interface/controller_interface.hpp>
#include <urdf/model.h>
#include <custom_msgs/msg/obstacle_array.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <kdl/frames.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <joint_limits/joint_limits.hpp>
#include <joint_limits/joint_limits_urdf.hpp>

namespace project {

double ProjectController::repulsiveAPF(double distance, double distance_prime) {
  if (distance >= threshold_) {
    return 0.0;
  }
  distance = std::max(distance, 0.01);
  return k_
      * (1.0 / distance - 1.0 / threshold_)
      * (1.0 / (distance * distance)) * distance_prime;
}

controller_interface::return_type ProjectController::update(
  const rclcpp::Time& /*time*/,
  const rclcpp::Duration& period) {
  // Update joint states from hardware state interface
  updateJointStates();

  const double dt = period.seconds();
  elapsed_time_ += dt;
  pat_state_elapsed_ += dt;

  // Save current state to KDL variables for solvers
  for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
    q_kdl_(i) = position_interface_vals_(i);
    q_dot_kdl_(i) = velocity_interface_vals_(i);
  }

  // Current EE pose
  solver_->computeFK(q_kdl_, x_kdl_);

  const std::int64_t marker_time_ns = last_valid_marker_ns_.load();
  const std::int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  const std::int64_t marker_age_ns = now_ns - marker_time_ns;
  const bool marker_available =
      marker_time_ns > 0 && marker_age_ns >= 0 &&
      marker_age_ns <= static_cast<std::int64_t>(marker_timeout_ * 1e9);
  const bool marker_detected = marker_detected_.load();

  // ================= ex5: visual servoing -> task targets =================
  if (marker_detected && marker_available) {
    T_cam_marker_ = *rt_marker_pose_buffer_.readFromRT()->get();
    T_base_marker_ = T_base_cam_ * T_cam_marker_;
  }

  const KDL::Rotation R_off = KDL::Rotation::RPY(
      wait_orientation_[0], wait_orientation_[1], wait_orientation_[2]);
  const auto target_at_offset = [this, &R_off](const std::vector<double>& offset) {
    return KDL::Frame(
        R_off,
        T_base_marker_.p + KDL::Vector(offset[0], offset[1], offset[2]));
  };
  const KDL::Frame hover_target = target_at_offset(wait_offset_);
  const KDL::Frame pat_target = target_at_offset(pat_offset_);

  switch (pat_state_) {
    case PatState::WAITING:
      if (marker_detected && marker_available && elapsed_time_ >= next_pat_time_) {
        manual_goal_available_.store(false);
        pat_state_ = PatState::APPROACHING;
        pat_state_elapsed_ = 0.0;
        xd_kdl_ = hover_target;
      } else if (manual_goal_available_.load()) {
        xd_kdl_ = *rt_goal_pose_buffer_.readFromRT()->get();
      }
      break;

    case PatState::APPROACHING:
      xd_kdl_ = hover_target;
      if (!marker_available) {
        pat_state_ = PatState::RETREATING;
        pat_state_elapsed_ = 0.0;
      } else if ((x_kdl_.p - hover_target.p).Norm() < approach_tolerance_) {
        pat_state_ = PatState::PATTING;
        pat_state_elapsed_ = 0.0;
        xd_kdl_ = pat_target;
      }
      break;

    case PatState::PATTING:
      if (!marker_available || pat_state_elapsed_ >= pat_duration_) {
        pat_state_ = PatState::RETREATING;
        pat_state_elapsed_ = 0.0;
        xd_kdl_ = hover_target;
      } else {
        xd_kdl_ = pat_target;
      }
      break;

    case PatState::RETREATING:
      xd_kdl_ = hover_target;
      if ((x_kdl_.p - hover_target.p).Norm() < approach_tolerance_ ||
          pat_state_elapsed_ >= retreat_timeout_) {
        pat_state_ = PatState::WAITING;
        pat_state_elapsed_ = 0.0;
        next_pat_time_ = elapsed_time_ + repeat_delay_;
      }
      break;
  }

  // ================= ex7: dynamics + Jacobian =================
  solver_->compute_dyn_params(q_kdl_, q_dot_kdl_, M_kdl_, G_kdl_, C_kdl_);
  solver_->compute_jac(q_kdl_, J_kdl_);
  solver_->compute_jac_dot(q_kdl_, q_dot_kdl_, J_dot_kdl_);

  M_ = M_kdl_.data;
  G_ = G_kdl_.data;
  C_ = C_kdl_.data;
  J_ = J_kdl_.data;
  J_dot_ = J_dot_kdl_.data;
  q_ = q_kdl_.data;
  q_dot_ = q_dot_kdl_.data;

  // ================= ex7: external wrench (compliance feedback) =================
  if (use_force_measurement_.load()) {
    if (use_fake_wrench_.load()) {
      h_e_ = *rt_fake_wrench_.readFromRT()->get();
    } else {
      h_e_ = *rt_force_torque_.readFromRT()->get();
    }
    // Low-pass filter force measurement in sensor frame
    h_e_ = h_e_old_ + 0.8 * (h_e_ - h_e_old_);
    h_e_old_ = h_e_;

    // Transform wrench from sensor frame to base frame using the Adjoint
    KDL::Vector p = x_kdl_.p;
    Eigen::Matrix<double, 3, 3> p_hat;
    p_hat << 0, -p.z(), p.y(),
             p.z(), 0, -p.x(),
            -p.y(), p.x(), 0;
    Eigen::Matrix<double, 3, 3> R;
    R << x_kdl_.M.data[0], x_kdl_.M.data[1], x_kdl_.M.data[2],
         x_kdl_.M.data[3], x_kdl_.M.data[4], x_kdl_.M.data[5],
         x_kdl_.M.data[6], x_kdl_.M.data[7], x_kdl_.M.data[8];
    Eigen::Matrix<double, 6, 6> Adj;
    Adj.block<3, 3>(0, 0) = R;
    Adj.block<3, 3>(0, 3).setZero();
    Adj.block<3, 3>(3, 0) = p_hat * R;
    Adj.block<3, 3>(3, 3) = R;
    h_e_ = Adj * h_e_;
  } else {
    h_e_ = Vector6d::Zero();
  }

  // ================= ex7: null-space stabilizing torque =================
  Eigen::MatrixXd M_inv = M_.inverse();
  Eigen::MatrixXd Lambda_inv = J_ * M_inv * J_.transpose();
  Eigen::MatrixXd J_bar = M_inv * J_.transpose() * Lambda_inv.inverse();
  Eigen::MatrixXd N = Eigen::MatrixXd::Identity(NUM_JOINTS, NUM_JOINTS)
                    - J_.transpose() * J_bar.transpose();
  tau_null_ = N.transpose() * (-0.75 * q_dot_);

  // Gains (same law as ex7)
  Md_.setIdentity();
  Md_ = Md_ * md_scale_;
  Kp_ = Md_ * (wn_ * wn_);
  Kd_ = 2.0 * zeta_ * (Kp_ * Md_).cwiseSqrt();

  // Position and velocity deltas in task space
  x_dot_ = J_ * q_dot_;
  x_dot_tilde_ = -x_dot_;
  KDL::Twist x_error = KDL::diff(x_kdl_, xd_kdl_);
  for (int i = 0; i < 6; ++i) {
    x_tilde_(i) = x_error(i);
  }

  // Impedance torque command (operational space)
  solver_->get_damped_pseudo_inverse(J_, J_pinv_);
  Vector7d u_ = J_pinv_ * Md_.inverse()
              * (Md_ * xd_ddot_ + Kd_ * x_dot_tilde_ + Kp_ * x_tilde_
                 - Md_ * (J_dot_ * q_dot_) - h_e_);
  tau_pos_ = M_ * u_ + C_ + G_;

  // ================= ex6: potential-field obstacle avoidance =================
  tau_rep_ = Vector7d::Zero();
  const auto& obstacles = *rt_obstacles_.readFromRT();
  for (const auto& obs : obstacles) {
    for (std::size_t frame_idx = 0; frame_idx < chain_.getNrOfSegments(); ++frame_idx) {
      KDL::Frame joint_pose;
      solver_->computeFK(q_kdl_, joint_pose, frame_idx);
      Eigen::Vector3d p(joint_pose.p.x(), joint_pose.p.y(), joint_pose.p.z());
      Eigen::Vector3d p_obs(obs.position.x, obs.position.y, obs.position.z);

      KDL::Jacobian J(NUM_JOINTS);
      solver_->compute_jac(q_kdl_, J, frame_idx);
      Eigen::Matrix<double, 3, NUM_JOINTS> Jp = J.data.topRows(3);
      const auto accumulate_repulsion = [this](
          double distance,
          const Eigen::RowVector<double, NUM_JOINTS>& distance_prime) {
        for (std::size_t j = 0; j < NUM_JOINTS; ++j) {
          tau_rep_(j) += repulsiveAPF(distance, distance_prime(j));
        }
      };

      // Spherical obstacle
      if (obs.shape_type == custom_msgs::msg::Obstacle::SPHERE) {
        const Eigen::Vector3d diff = p - p_obs;
        const double center_distance = diff.norm();
        Eigen::Vector3d direction = Eigen::Vector3d::UnitX();
        if (center_distance > 1e-6) {
          direction = diff / center_distance;
        }
        const double clearance = center_distance - obs.radius;
        const Eigen::RowVector<double, NUM_JOINTS> clearance_prime = direction.transpose() * Jp;
        accumulate_repulsion(clearance, clearance_prime);
      }
      // Box obstacle (big box on the conveyor)
      else if (obs.shape_type == custom_msgs::msg::Obstacle::BOX) {
        Eigen::Vector3d box_size(obs.box_size.at(0), obs.box_size.at(1), obs.box_size.at(2));
        Eigen::Vector3d half_size = box_size * 0.5;
        const Eigen::Vector3d local = p - p_obs;
        const bool inside =
            (local.cwiseAbs().array() <= half_size.array()).all();
        if (inside) {
          Eigen::Index axis = 0;
          double face_clearance = half_size.x() - std::abs(local.x());
          for (Eigen::Index i = 1; i < 3; ++i) {
            const double candidate = half_size(i) - std::abs(local(i));
            if (candidate < face_clearance) {
              axis = i;
              face_clearance = candidate;
            }
          }
          Eigen::Vector3d outward = Eigen::Vector3d::Zero();
          outward(axis) = local(axis) >= 0.0 ? 1.0 : -1.0;
          const Eigen::RowVector<double, NUM_JOINTS> clearance_prime = outward.transpose() * Jp;
          accumulate_repulsion(-face_clearance, clearance_prime);
        } else {
          const Eigen::Vector3d closest = local.cwiseMax(-half_size).cwiseMin(half_size);
          const Eigen::Vector3d delta = local - closest;
          const double distance = delta.norm();
          const Eigen::RowVector<double, NUM_JOINTS> clearance_prime =
              delta.transpose() * Jp / std::max(distance, 1e-6);
          accumulate_repulsion(distance, clearance_prime);
        }
      }
    }
  }

  // ================= final torque command =================
  tau_cmd_ = tau_pos_ + tau_null_ + tau_rep_;

  // Send torque commands to the hardware command interface
  for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
    command_interfaces_[i].set_value(tau_cmd_(i));
  }

  publish_markers();
  publish_goal_tf();

  return controller_interface::return_type::OK;
}

CallbackReturn ProjectController::on_init() {
  try {
    joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
    command_interface_types_ = auto_declare<std::vector<std::string>>(
        "command_interfaces", command_interface_types_);
    state_interface_types_ = auto_declare<std::vector<std::string>>(
        "state_interfaces", state_interface_types_);
    root_link_ = auto_declare<std::string>("root_link", root_link_);
    tip_link_ = auto_declare<std::string>("tip_link", tip_link_);
    p_gains_ = auto_declare<std::vector<double>>("p_gains", p_gains_);
    d_gains_ = auto_declare<std::vector<double>>("d_gains", d_gains_);
    camera_pose_ = auto_declare<std::vector<double>>("camera_pose", camera_pose_);
    wait_offset_ = auto_declare<std::vector<double>>("wait_offset", wait_offset_);
    pat_offset_ = auto_declare<std::vector<double>>("pat_offset", pat_offset_);
    wait_orientation_ = auto_declare<std::vector<double>>("wait_orientation", wait_orientation_);
    marker_timeout_ = auto_declare<double>("marker_timeout", marker_timeout_);
    approach_tolerance_ = auto_declare<double>("approach_tolerance", approach_tolerance_);
    pat_duration_ = auto_declare<double>("pat_duration", pat_duration_);
    repeat_delay_ = auto_declare<double>("repeat_delay", repeat_delay_);
    retreat_timeout_ = auto_declare<double>("retreat_timeout", retreat_timeout_);
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage (on_init) with message: %s\n", e.what());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_node()->get_logger(), "Robot initialization done.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn ProjectController::on_configure(
  const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(get_node()->get_logger(), "Robot configuration started");

  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "/robot_state_publisher");
  parameters_client->wait_for_service();
  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();

  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
    return CallbackReturn::ERROR;
  }

  urdf::Model urdf;
  if (!urdf.initString(robot_description_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse urdf file");
    return CallbackReturn::ERROR;
  } else {
    RCLCPP_INFO(get_node()->get_logger(), "Found robot_description");
  }

  joint_min_.resize(NUM_JOINTS);
  joint_max_.resize(NUM_JOINTS);
  q_kdl_.resize(NUM_JOINTS);
  q_dot_kdl_.resize(NUM_JOINTS);
  gravity_.resize(NUM_JOINTS);
  q_desired_.resize(NUM_JOINTS);

  for (std::size_t i = 0; i < NUM_JOINTS; i++) {
    urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_names_[i]);
    if (!joint_urdf) {
      RCLCPP_ERROR(get_node()->get_logger(), "Could not find joint '%s' in urdf", joint_names_[i].c_str());
      return CallbackReturn::ERROR;
    }
    joint_limits::JointLimits limits;
    if (!joint_limits::getJointLimits(joint_urdf, limits)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Could not get limits for joint '%s'", joint_names_[i].c_str());
      return CallbackReturn::ERROR;
    }
    joint_min_(i) = limits.min_position;
    joint_max_(i) = limits.max_position;
  }

  if (!kdl_parser::treeFromUrdfModel(urdf, tree_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to construct a KDL tree.");
    return CallbackReturn::ERROR;
  } else {
    RCLCPP_INFO(get_node()->get_logger(), "Constructed a KDL tree.");
  }

  if (!tree_.getChain(root_link_, tip_link_, chain_)) {
    kdl_chain_error_print();
    return CallbackReturn::ERROR;
  } else {
    kdl_chain_success_print();
  }

  if (camera_pose_.size() != 6) {
    RCLCPP_ERROR(get_node()->get_logger(), "camera_pose must contain x, y, z, roll, pitch, yaw.");
    return CallbackReturn::ERROR;
  }

  solver_.reset(new Solver(chain_));
  T_base_cam_ = KDL::Frame(
      KDL::Rotation::RPY(camera_pose_[3], camera_pose_[4], camera_pose_[5]),
      KDL::Vector(camera_pose_[0], camera_pose_[1], camera_pose_[2]));

  goal_pose_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg) {
        goal_pose_callback(msg);
      });

  marker_pose_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      "marker_pose", rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<std_msgs::msg::Float64MultiArray> msg) {
        marker_pose_callback(msg);
      });

  init_obstacle_subscriber_ = get_node()->create_subscription<custom_msgs::msg::ObstacleArray>(
      "obstacles", rclcpp::QoS(1).transient_local(),
      [this](const std::shared_ptr<custom_msgs::msg::ObstacleArray> msg) {
        init_obstacle_callback(msg);
      });

  sub_obj_pose_ = get_node()->create_subscription<tf2_msgs::msg::TFMessage>(
      "world/project_world/pose/info", rclcpp::SystemDefaultsQoS(),
      [this](const tf2_msgs::msg::TFMessage& msg) {
        object_pose_callback(msg);
      });

  force_torque_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "sensor_joint/force_torque", rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<geometry_msgs::msg::WrenchStamped> msg) {
        force_torque_callback(msg);
      });
  fake_wrench_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "sensor_joint/force_torque/fake", rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<geometry_msgs::msg::WrenchStamped> msg) {
        fake_wrench_callback(msg);
      });

  parameter_callback_handle_ = get_node()->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto& param : parameters) {
          if (param.get_name() == "use_force_measurement") {
            use_force_measurement_.store(param.as_bool());
            RCLCPP_INFO(get_node()->get_logger(), "use_force_measurement set to %s",
                        param.as_bool() ? "true" : "false");
          } else if (param.get_name() == "use_fake_wrench") {
            use_fake_wrench_.store(param.as_bool());
            RCLCPP_INFO(get_node()->get_logger(), "use_fake_wrench set to %s",
                        param.as_bool() ? "true" : "false");
          }
        }
        return result;
      });

  pub_markers_ = get_node()->create_publisher<visualization_msgs::msg::MarkerArray>("markers", 1000);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(get_node());

  RCLCPP_INFO(get_node()->get_logger(), "Robot configuration done.");
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
ProjectController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& jname : joint_names_) {
    for (const auto& interface_type : command_interface_types_) {
      config.names.push_back(jname + "/" + interface_type);
    }
  }
  return config;
}

controller_interface::InterfaceConfiguration
ProjectController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& jname : joint_names_) {
    for (const auto& interface_type : state_interface_types_) {
      config.names.push_back(jname + "/" + interface_type);
    }
  }
  return config;
}

CallbackReturn ProjectController::on_activate(
  const rclcpp_lifecycle::State& /*previous_state*/) {
  updateJointStates();
  elapsed_time_ = 0.0;

  M_kdl_.resize(NUM_JOINTS);
  G_kdl_.resize(NUM_JOINTS);
  C_kdl_.resize(NUM_JOINTS);
  J_kdl_.resize(NUM_JOINTS);
  J_dot_kdl_.resize(NUM_JOINTS);
  q_kdl_.resize(NUM_JOINTS);
  q_dot_kdl_.resize(NUM_JOINTS);
  for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
    q_kdl_(i) = position_interface_vals_(i);
    q_dot_kdl_(i) = velocity_interface_vals_(i);
  }

  Md_.setIdentity();
  Md_ = Md_ * md_scale_;
  Kp_.setZero();
  Kd_.setZero();

  if (solver_->computeFK(q_kdl_, x_kdl_) < 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize the hold pose.");
    return CallbackReturn::ERROR;
  }
  xd_kdl_ = x_kdl_;
  auto xd = std::make_shared<KDL::Frame>(x_kdl_);
  rt_goal_pose_buffer_.writeFromNonRT(xd);
  T_base_marker_ = KDL::Frame::Identity();
  pat_state_ = PatState::WAITING;
  pat_state_elapsed_ = 0.0;
  next_pat_time_ = elapsed_time_;
  marker_detected_.store(false);
  last_valid_marker_ns_.store(0);
  manual_goal_available_.store(false);

  Vector6d h_e = Vector6d::Zero();
  auto h_e_ptr = std::make_shared<Vector6d>(h_e);
  rt_force_torque_.writeFromNonRT(h_e_ptr);
  rt_fake_wrench_.writeFromNonRT(h_e_ptr);
  h_e_old_ = Vector6d::Zero();

  pub_markers_->on_activate();

  return CallbackReturn::SUCCESS;
}

void ProjectController::updateJointStates() {
  if (state_interfaces_.size() != state_interface_types_.size() * NUM_JOINTS) {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid number of state interfaces");
    return;
  }
  auto* interfaces = state_interfaces_.data();
  for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
    const auto& position_interface = interfaces[2 * i];
    const auto& velocity_interface = interfaces[2 * i + 1];
    const auto& pos_name = position_interface.get_interface_name();
    if (pos_name != "position") {
      RCLCPP_ERROR(get_node()->get_logger(), "Expected position interface, but got %s", pos_name.c_str());
      return;
    }
    const auto& vel_name = velocity_interface.get_interface_name();
    if (vel_name != "velocity") {
      RCLCPP_ERROR(get_node()->get_logger(), "Expected velocity interface, but got %s", vel_name.c_str());
      return;
    }
    position_interface_vals_(i) = position_interface.get_value();
    velocity_interface_vals_(i) = velocity_interface.get_value();
  }
}

// Goal pose subscriber callback (manual override)
void ProjectController::goal_pose_callback(const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg) {
  auto xd = std::make_shared<KDL::Frame>();
  xd->p = KDL::Vector(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  xd->M = KDL::Rotation::Quaternion(
      msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w);
  rt_goal_pose_buffer_.writeFromNonRT(xd);
  manual_goal_available_.store(true);
}

// Marker pose in camera optical frame (visual servoing input)
void ProjectController::marker_pose_callback(const std::shared_ptr<std_msgs::msg::Float64MultiArray> msg) {
  if (msg->data.empty()) {
    marker_detected_.store(false);
    return;
  }
  if (msg->data.size() != 12) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Ignoring marker pose with %zu values; expected 12.", msg->data.size());
    marker_detected_.store(false);
    return;
  }

  {
    double xx, xy, xz, yx, yy, yz, zx, zy, zz, tx, ty, tz;
    xx = msg->data[0], xy = msg->data[1], xz = msg->data[2];
    yx = msg->data[3], yy = msg->data[4], yz = msg->data[5];
    zx = msg->data[6], zy = msg->data[7], zz = msg->data[8];
    tx = msg->data[9], ty = msg->data[10], tz = msg->data[11];
    auto T_cam_marker = std::make_shared<KDL::Frame>(
        KDL::Frame(KDL::Rotation(xx, xy, xz, yx, yy, yz, zx, zy, zz), KDL::Vector(tx, ty, tz)));
    rt_marker_pose_buffer_.writeFromNonRT(T_cam_marker);
    last_valid_marker_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
    marker_detected_.store(true);
  }
}

// Obstacle initial data (shape + name) from scene
void ProjectController::init_obstacle_callback(const std::shared_ptr<custom_msgs::msg::ObstacleArray> msg) {
  obstacle_map_.clear();
  std::vector<custom_msgs::msg::Obstacle> obs_vec;
  for (auto obs : msg->obstacles) {
    obstacle_map_.emplace(obs.name, obs);
    obs_vec.push_back(obs);
  }
  rt_obstacles_.writeFromNonRT(obs_vec);
}

// Obstacle live poses from Gazebo TF (scene broadcaster)
void ProjectController::object_pose_callback(const tf2_msgs::msg::TFMessage& msg) {
  for (geometry_msgs::msg::TransformStamped t : msg.transforms) {
    auto it = obstacle_map_.find(t.child_frame_id);
    if (it != obstacle_map_.end()) {
      it->second.position.x = t.transform.translation.x;
      it->second.position.y = t.transform.translation.y;
      it->second.position.z = t.transform.translation.z;
      it->second.orientation = t.transform.rotation;
    }
  }
  std::vector<custom_msgs::msg::Obstacle> snapshot;
  snapshot.reserve(obstacle_map_.size());
  for (const auto& [name, obs] : obstacle_map_) {
    snapshot.push_back(obs);
  }
  rt_obstacles_.writeFromNonRT(snapshot);
}

void ProjectController::force_torque_callback(const std::shared_ptr<geometry_msgs::msg::WrenchStamped> msg) {
  Vector6d h_e(msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
               msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z);
  auto h_e_ptr = std::make_shared<Vector6d>(h_e);
  rt_force_torque_.writeFromNonRT(h_e_ptr);
}

void ProjectController::fake_wrench_callback(const std::shared_ptr<geometry_msgs::msg::WrenchStamped> msg) {
  Vector6d h_e(msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
               msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z);
  auto h_e_ptr = std::make_shared<Vector6d>(h_e);
  rt_fake_wrench_.writeFromNonRT(h_e_ptr);
}

void ProjectController::publish_markers() {
  visualization_msgs::msg::MarkerArray msg;
  const auto& obstacles = *rt_obstacles_.readFromRT();
  int i = 0;
  for (const auto& obs : obstacles) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base";
    marker.header.stamp = get_node()->get_clock()->now();
    marker.ns = "";
    marker.id = i++;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = obs.position;
    marker.pose.orientation = obs.orientation;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = rclcpp::Duration::from_nanoseconds(0);
    if (obs.shape_type == custom_msgs::msg::Obstacle::SPHERE) {
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.scale.x = 2 * obs.radius;
      marker.scale.y = 2 * obs.radius;
      marker.scale.z = 2 * obs.radius;
    } else if (obs.shape_type == custom_msgs::msg::Obstacle::BOX) {
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.scale.x = obs.box_size.at(0);
      marker.scale.y = obs.box_size.at(1);
      marker.scale.z = obs.box_size.at(2);
    }
    msg.markers.push_back(marker);
  }
  pub_markers_->publish(msg);
}

void ProjectController::publish_goal_tf() {
  geometry_msgs::msg::TransformStamped t;
  KDL::Frame f = xd_kdl_;
  t.header.stamp = get_node()->get_clock()->now();
  t.header.frame_id = "fr3_link0";
  t.child_frame_id = "goal_pose";
  t.transform.translation.x = f.p.x();
  t.transform.translation.y = f.p.y();
  t.transform.translation.z = f.p.z();
  double qx, qy, qz, qw;
  f.M.GetQuaternion(qx, qy, qz, qw);
  t.transform.rotation.x = qx;
  t.transform.rotation.y = qy;
  t.transform.rotation.z = qz;
  t.transform.rotation.w = qw;
  tf_broadcaster_->sendTransform(t);
}

void ProjectController::kdl_chain_error_print() {
  RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Failed to get KDL chain from tree:");
  RCLCPP_ERROR_STREAM(get_node()->get_logger(), "  " << root_link_ << " --> " << tip_link_);
  RCLCPP_ERROR_STREAM(get_node()->get_logger(), "  Tree has " << tree_.getNrOfJoints() << " joints");
  RCLCPP_ERROR_STREAM(get_node()->get_logger(), "  Tree has " << tree_.getNrOfSegments() << " segments");
  KDL::SegmentMap segment_map = tree_.getSegments();
  for (KDL::SegmentMap::iterator it = segment_map.begin(); it != segment_map.end(); it++) {
    RCLCPP_ERROR(get_node()->get_logger(), "    %s", std::string((*it).first).c_str());
  }
}

void ProjectController::kdl_chain_success_print() {
  RCLCPP_INFO(get_node()->get_logger(), "Got kdl chain");
  RCLCPP_INFO(get_node()->get_logger(), "  %s --> %s", root_link_.c_str(), tip_link_.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "  Tree has %d joints", tree_.getNrOfJoints());
  RCLCPP_INFO(get_node()->get_logger(), "  Tree has %d segments", tree_.getNrOfSegments());
  RCLCPP_INFO(get_node()->get_logger(), "  Chain has %d joints", chain_.getNrOfJoints());
  RCLCPP_INFO(get_node()->get_logger(), "  Chain has %d segments", chain_.getNrOfSegments());
}

}  // namespace project
