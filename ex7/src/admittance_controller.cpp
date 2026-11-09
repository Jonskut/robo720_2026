#include "ex7/admittance_controller.hpp"
#include "franka_kdl/robot_constants.hpp"
#include "franka_kdl/solver.hpp"

#include <algorithm>
#include <cstddef>
#include <exception>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <controller_interface/controller_interface.hpp>
#include <urdf/model.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <kdl/frames.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>


namespace ex7 {


controller_interface::return_type AdmittanceController::update(
  const rclcpp::Time& /*time*/,
  const rclcpp::Duration& period) {
    // Update joint states from hardware state interface
    updateJointStates();

    const double dt = period.seconds();
    elapsed_time_ = elapsed_time_ + dt;

    // Save current state to KDL variables for solvers
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        q_kdl_(i) = position_interface_vals_(i);
        q_dot_kdl_(i) = velocity_interface_vals_(i);
    }

    // Current EE pose
    solver_->computeFK(q_kdl_, x_kdl_);

    // Read the goal pose and convert it into KDL Frame
    xd_kdl_ = *rt_goal_pose_buffer_.readFromRT()->get();

    // Read the force measurement
    if (use_fake_wrench_.load()) {
        h_e_ = *rt_fake_wrench_.readFromRT()->get();
    }
    else {
        h_e_ = *rt_force_torque_.readFromRT()->get();
    }

    // Low-pass filter force measurement in sensor frame
    h_e_ = h_e_old_ + 0.8 * (h_e_ - h_e_old_);
    h_e_old_ = h_e_;

    /**
     * TODO: Rotate from sensor frame to desired pose frame.
     * hint: we can do two separate rotations, like when chaining transforms
     */

    /**
     * TODO: Compute the compliant frame.
     * hint: use equations 11-13 from the assignment to get z_tilde and integrate
     * to get compliant frame with code snippets shown in assignment.
     * You can also use ODE solver to get z_tilde.
     */

    /**
     * TODO: Compute error in task space.
     */

    // Get dynamics parameters and Jacobian
    solver_->compute_dyn_params(q_kdl_, q_dot_kdl_, M_kdl_, G_kdl_, C_kdl_);
    solver_->compute_jac(q_kdl_, J_kdl_);
    solver_->compute_jac_dot(q_kdl_, q_dot_kdl_, J_dot_kdl_);

    // Copy data to Eigen variables for matrix computations
    M_ = M_kdl_.data;
    G_ = G_kdl_.data;
    C_ = C_kdl_.data;
    J_ = J_kdl_.data;
    J_dot_ = J_dot_kdl_.data;
    q_ = q_kdl_.data;
    q_dot_ = q_dot_kdl_.data;

    // Jacobian pseudo-inverse
    solver_->get_damped_pseudo_inverse(J_, J_pinv_);

    // Null space stabilizing torque
    // 7DOF manipulator is redundant so some of the joints might move even
    // when the end effector is in the goal pose. tau_null prohibits this
    // without affecting the pose tracking
    Eigen::MatrixXd M_inv = M_.inverse();
    Eigen::MatrixXd Lambda_inv = J_ * M_inv * J_.transpose();
    Eigen::MatrixXd J_bar = M_inv * J_.transpose() * Lambda_inv.inverse();
    Eigen::MatrixXd N = Eigen::MatrixXd::Identity(NUM_JOINTS, NUM_JOINTS) - J_.transpose() * J_bar.transpose();
    tau_null_ = N.transpose() * (-0.75 * q_dot_);

    // Compute torque command
    /**
     * TODO: compute tau_pos_ with inverse dynamics controller
     */
    tau_ = tau_pos_ + tau_null_;

    // Send torque commands to the hardware command interface
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        command_interfaces_[i].set_value(tau_(i));
    }

    return controller_interface::return_type::OK;
}

CallbackReturn AdmittanceController::on_init() {
    // Get parameters from franka_controllers.yaml
    try {
        joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
        command_interface_types_ = auto_declare<std::vector<std::string>>(
            "command_interfaces", command_interface_types_);
        state_interface_types_ = auto_declare<std::vector<std::string>>(
            "state_interfaces", state_interface_types_);

        root_link_ = auto_declare<std::string>("root_link", root_link_);
        tip_link_ = auto_declare<std::string>("tip_link", tip_link_);

        auto_declare<bool>("use_fake_wrench", true);
    } 
    catch (const std::exception& e) {
        fprintf(stderr, "Exception thrown during init stage (on_init) with message: %s \n", e.what());
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_node()->get_logger(), "Robot initialization done.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn AdmittanceController::on_configure(
  const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(get_node()->get_logger(), "Robot configuration started");

    // Get the robot description parameter (effort_fr3_arm.urdf.xacro parsed into one URDF as a string)
    // Due to AsyncParametersClient, this has to be in on_configure() method.
    auto parameters_client =
        std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "/robot_state_publisher");
    parameters_client->wait_for_service();
    auto future = parameters_client->get_parameters({"robot_description"});
    auto result = future.get();

    // Check if robot description was retrieved succesfully
    if (!result.empty()) {
        robot_description_ = result[0].value_to_string();
    }
    else {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
        return CallbackReturn::ERROR;
    }

    // Parse URDF string into URDF object
    urdf::Model urdf;
    if (!urdf.initString(robot_description_)) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse urdf file");
        return CallbackReturn::ERROR;
    }
    else {
        RCLCPP_INFO(get_node()->get_logger(), "Found robot_description");
    }

    // Construct a KDL tree object from the URDF
    if (!kdl_parser::treeFromUrdfModel(urdf, tree_)) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to construct a KDL tree.");
        return CallbackReturn::ERROR;
    }
    else {
        RCLCPP_INFO(get_node()->get_logger(), "Constructed a KDL tree.");
    }

    // Get the KDL chain from KDL tree (for KDL Solver objects)
    if (!tree_.getChain(root_link_, tip_link_, chain_)) {
        kdl_chain_error_print();

        return CallbackReturn::ERROR;
    }
    else {
        kdl_chain_success_print();
    }

    // Create a Solver instance
    solver_.reset(new Solver(chain_));

    // Create a subscription for the joint command
    goal_pose_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose", rclcpp::SystemDefaultsQoS(), std::bind(&AdmittanceController::goal_pose_callback,
        this, std::placeholders::_1));

    force_torque_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "sensor_joint/force_torque", rclcpp::SystemDefaultsQoS(),
        std::bind(&AdmittanceController::force_torque_callback, this, std::placeholders::_1));
    fake_wrench_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "sensor_joint/force_torque/fake", rclcpp::SystemDefaultsQoS(),
        std::bind(&AdmittanceController::fake_wrench_callback, this, std::placeholders::_1));

    // Parameter callback for using force measurement
    parameter_callback_handle_ = get_node()->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto& param : parameters) {
            if (param.get_name() == "use_fake_wrench") {
                use_fake_wrench_.store(param.as_bool());
                RCLCPP_INFO(get_node()->get_logger(), "use_fake_wrench set to %s",
                            param.as_bool() ? "true" : "false");
            }
        }
        return result;
    });

    RCLCPP_INFO(get_node()->get_logger(), "Robot configuration done.");
    return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
AdmittanceController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto & jname : joint_names_) {
        for (const auto & interface_type : command_interface_types_) {
            config.names.push_back(jname + "/" + interface_type);
        }
    }
    return config;
}

controller_interface::InterfaceConfiguration
AdmittanceController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto & jname : joint_names_) {
        for (const auto & interface_type : state_interface_types_) {
            config.names.push_back(jname + "/" + interface_type);
        }
    }
    return config;
}

CallbackReturn AdmittanceController::on_activate(
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

    // Initialize realtime buffer
    auto xd = std::make_shared<KDL::Frame>();
    xd->p = KDL::Vector(0.5, 0.0, 1.6);
    xd->M = KDL::Rotation::EulerZYX(0.0, 0.0, M_PI);
    rt_goal_pose_buffer_.writeFromNonRT(xd);

    Vector6d h_e = Vector6d::Zero();
    auto h_e_ptr = std::make_shared<Vector6d>(h_e);
    rt_force_torque_.writeFromNonRT(h_e_ptr);
    rt_fake_wrench_.writeFromNonRT(h_e_ptr);
    h_e_old_ = Vector6d::Zero();

    return CallbackReturn::SUCCESS;
}

void AdmittanceController::updateJointStates() {
    // Pre-check array size to avoid bounds checking in loop
    if (state_interfaces_.size() != state_interface_types_.size() * NUM_JOINTS) {
        RCLCPP_ERROR(get_node()->get_logger(), "Invalid number of state interfaces");
        return;
    }

    // Get the current joint positions
    auto* interfaces = state_interfaces_.data();
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        // Access interfaces directly with pointer arithmetic
        const auto& position_interface = interfaces[2*i];
        const auto& velocity_interface = interfaces[2*i + 1];

        // Interface name comparison
        const auto& pos_name = position_interface.get_interface_name();
        if (pos_name != "position") {
            RCLCPP_ERROR(get_node()->get_logger(), "Expected position interface, but got %s",
                         pos_name.c_str());
            return;
        }
        const auto& vel_name = velocity_interface.get_interface_name();
        if (vel_name != "velocity") {
            RCLCPP_ERROR(get_node()->get_logger(), "Expected velocity interface, but got %s",
                         vel_name.c_str());
            return;
        }
        
        // Direct value assignment
        position_interface_vals_(i) = position_interface.get_value();
        velocity_interface_vals_(i) = velocity_interface.get_value();
    }
}

// Goal pose subscriber callback
void AdmittanceController::goal_pose_callback(const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg) {
    auto xd = std::make_shared<KDL::Frame>();

    xd->p = KDL::Vector(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    xd->M = KDL::Rotation::Quaternion(
        msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);

    rt_goal_pose_buffer_.writeFromNonRT(xd);
}

// Force torque subscriber callback
void AdmittanceController::force_torque_callback(const std::shared_ptr<geometry_msgs::msg::WrenchStamped> msg) {
    Vector6d h_e(msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
                 msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z);
    auto h_e_ptr = std::make_shared<Vector6d>(h_e);
    rt_force_torque_.writeFromNonRT(h_e_ptr);
}

void AdmittanceController::fake_wrench_callback(const std::shared_ptr<geometry_msgs::msg::WrenchStamped> msg) {
    Vector6d h_e(msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
                 msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z);
    auto h_e_ptr = std::make_shared<Vector6d>(h_e);
    rt_fake_wrench_.writeFromNonRT(h_e_ptr);
}

// Debug print functions to reduce clutter in controller code
void AdmittanceController::kdl_chain_error_print() {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Failed to get KDL chain from tree: ");
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "  " << root_link_ << " --> " << tip_link_);
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "  Tree has " << tree_.getNrOfJoints() << " joints");
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "  Tree has " << tree_.getNrOfSegments() << " segments");
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "  The segments are:");

    KDL::SegmentMap segment_map = tree_.getSegments();
    KDL::SegmentMap::iterator it;

    for (it = segment_map.begin(); it != segment_map.end(); it++) {
        RCLCPP_ERROR(get_node()->get_logger(), "    %s", std::string((*it).first).c_str());
    }
}

void AdmittanceController::kdl_chain_success_print() {
    RCLCPP_INFO(get_node()->get_logger(), "Got kdl chain");

    // debug: print kdl tree and kdl chain
    RCLCPP_INFO(get_node()->get_logger(), "  %s --> %s", root_link_.c_str(), tip_link_.c_str());
    RCLCPP_INFO(get_node()->get_logger(), "  Tree has %d joints", tree_.getNrOfJoints());
    RCLCPP_INFO(get_node()->get_logger(), "  Tree has %d segments", tree_.getNrOfSegments());
    RCLCPP_INFO(get_node()->get_logger(), "  The kdl_tree_ segments are:");

    // Print the segments of the KDL tree
    KDL::SegmentMap segment_map = tree_.getSegments();
    KDL::SegmentMap::iterator it;
    for (it = segment_map.begin(); it != segment_map.end(); it++)
    {
    RCLCPP_INFO(get_node()->get_logger(), "    %s", std::string((*it).first).c_str());
    }
    RCLCPP_INFO(get_node()->get_logger(), "  Chain has %d joints", chain_.getNrOfJoints());
    RCLCPP_INFO(get_node()->get_logger(), "  Chain has %d segments", chain_.getNrOfSegments());
    RCLCPP_INFO(get_node()->get_logger(), "  The kdl_chain_ segments are:");
    for (unsigned int i = 0; i < chain_.getNrOfSegments(); i++) {
        const KDL::Segment& segment = chain_.getSegment(i);
        RCLCPP_INFO(get_node()->get_logger(), "    %s", segment.getName().c_str());
    }
}

}  // namespace ex7
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ex7::AdmittanceController,
                       controller_interface::ControllerInterface)