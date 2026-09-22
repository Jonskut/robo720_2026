#include "ex4/task_space_kinematic_controller.hpp"
#include "franka_kdl/robot_constants.hpp"
#include "franka_kdl/solver.hpp"

#include <cstddef>
#include <exception>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <controller_interface/controller_interface.hpp>
#include <urdf/model.h>

#include <kdl/frames.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>


namespace ex4 {


controller_interface::return_type TaskSpaceKinematicController::update(
  const rclcpp::Time& /*time*/,
  const rclcpp::Duration& period) {
    // Update joint states from hardware state interface
    updateJointStates();

    elapsed_time_ = elapsed_time_ + period.seconds();

    // Save current state to KDL variables for solvers
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        q_kdl_(i) = position_interface_vals_(i);
    }

    // Read the goal pose and convert it into KDL Frame
    auto xd = *rt_goal_pose_buffer_.readFromRT();
    KDL::Vector pos_d(xd->pose.position.x, xd->pose.position.y, xd->pose.position.z);
    KDL::Rotation rot_d = KDL::Rotation::Quaternion(
        xd->pose.orientation.x, xd->pose.orientation.y, xd->pose.orientation.z, xd->pose.orientation.w);
    KDL::Frame pose_d(rot_d, pos_d);

    // Task 4
    // Solve the jacobian and invert it
    int result = solver_->compute_jac(
        q_kdl_,
        jac_,
        -1);
    
    if (result < 0) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Failed to compute jacobian: %d", result);
        return controller_interface::return_type::ERROR;
    }

    Eigen::Matrix<double, NUM_TASK, NUM_JOINTS> jac_eigen;

    for (std::size_t row = 0; row < NUM_TASK; ++row) {
        for (std::size_t column = 0; column < NUM_JOINTS; ++column) {
            jac_eigen(row, column) = jac_.data(row, column);
        }
    }

    solver_->get_damped_pseudo_inverse(jac_eigen, jac_pinv_);

    // Solve the forward kinematics
    result = solver_->computeFK(
        q_kdl_,
        pose_e_);
    
    if (result < 0) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Failed to compute forward kinematics: %d", result);
        return controller_interface::return_type::ERROR;
    }

    // Modify pose error
    const KDL::Twist pose_error = KDL::diff(pose_e_, pose_d);

    Vector6d pose_error_eigen;
    pose_error_eigen << pose_error.vel.x(),
                        pose_error.vel.y(),
                        pose_error.vel.z(),
                        pose_error.rot.x(),
                        pose_error.rot.y(),
                        pose_error.rot.z();
   
    // Velocity command
    q_dot_cmd_ = Kp_ * jac_pinv_ * pose_error_eigen;

    // Send velocity commands to the low-level controller
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        command_interfaces_[i].set_value(q_dot_cmd_(i));
    }

    return controller_interface::return_type::OK;
}

CallbackReturn TaskSpaceKinematicController::on_init() {
    // Get parameters from franka_controllers.yaml
    try {
        joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
        command_interface_types_ = auto_declare<std::vector<std::string>>(
            "command_interfaces", command_interface_types_);
        state_interface_types_ = auto_declare<std::vector<std::string>>(
            "state_interfaces", state_interface_types_);

        following_controller_ = auto_declare<std::string>("following_controller", following_controller_);

        root_link_ = auto_declare<std::string>("root_link", root_link_);
        tip_link_ = auto_declare<std::string>("tip_link", tip_link_);
    } 
    catch (const std::exception& e) {
        fprintf(stderr, "Exception thrown during init stage (on_init) with message: %s \n", e.what());
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_node()->get_logger(), "Robot initialization done.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn TaskSpaceKinematicController::on_configure(
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

    // Create a subscription for the goal pose
    goal_pose_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose", rclcpp::SystemDefaultsQoS(), std::bind(&TaskSpaceKinematicController::goal_pose_callback,
        this, std::placeholders::_1));

    RCLCPP_INFO(get_node()->get_logger(), "Robot configuration done.");
    return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
TaskSpaceKinematicController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto & jname : joint_names_) {
        for (const auto & interface_type : command_interface_types_) {
            // Notice here the namespace in front of the command interface
            config.names.push_back(following_controller_ + "/" + jname + "/" + interface_type);
        }
    }
    return config;
}

controller_interface::InterfaceConfiguration
TaskSpaceKinematicController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto & jname : joint_names_) {
        for (const auto & interface_type : state_interface_types_) {
            config.names.push_back(jname + "/" + interface_type);
        }
    }
    return config;
}

CallbackReturn TaskSpaceKinematicController::on_activate(
  const rclcpp_lifecycle::State& /*previous_state*/) {
    updateJointStates();
    elapsed_time_ = 0.0;

    q_kdl_.resize(NUM_JOINTS);
    jac_.resize(NUM_JOINTS);

    // Initialize realtime buffer
    auto msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
    msg->pose.position.x = 0.5;
    msg->pose.position.y = 0.0;
    msg->pose.position.z = 1.6;
    // Orientation in Pose message is a quaternion, (0,0,0,1) is an identity quaternion
    msg->pose.orientation.x = 0.0;
    msg->pose.orientation.y = 0.0;
    msg->pose.orientation.z = 0.0;
    msg->pose.orientation.w = 1.0;
    rt_goal_pose_buffer_.writeFromNonRT(msg);

    return CallbackReturn::SUCCESS;
}

void TaskSpaceKinematicController::updateJointStates() {
    // Pre-check array size to avoid bounds checking in loop
    if (state_interfaces_.size() != state_interface_types_.size() * NUM_JOINTS) {
        RCLCPP_ERROR(get_node()->get_logger(), "Invalid number of state interfaces");
        return;
    }

    // Get the current joint positions
    auto* interfaces = state_interfaces_.data();
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        // Access interfaces directly with pointer arithmetic
        const auto& position_interface = interfaces[i];

        // Interface name comparison
        const auto& pos_name = position_interface.get_interface_name();
        if (pos_name != "position") {
            RCLCPP_ERROR(get_node()->get_logger(), "Expected position interface, but got %s",
                         pos_name.c_str());
            return;
        }
        
        // Direct value assignment
        position_interface_vals_(i) = position_interface.get_value();
    }
}

// Joint command subscriber callback
void TaskSpaceKinematicController::goal_pose_callback(const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg) {
    rt_goal_pose_buffer_.writeFromNonRT(msg);
}

// Debug print functions to reduce clutter in controller code
void TaskSpaceKinematicController::kdl_chain_error_print() {
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

void TaskSpaceKinematicController::kdl_chain_success_print() {
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

}  // namespace ex4
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ex4::TaskSpaceKinematicController,
                       controller_interface::ControllerInterface)