#include "ex6/potential_field_controller.hpp"
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
#include <custom_msgs/msg/obstacle_array.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <kdl/frames.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>


namespace ex6 {


controller_interface::return_type PotentialFieldController::update(
  const rclcpp::Time& /*time*/,
  const rclcpp::Duration& period) {
    // Update joint states from hardware state interface
    updateJointStates();

    elapsed_time_ = elapsed_time_ + period.seconds();

    // Save current state to KDL variables for solvers
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        q_kdl_(i) = position_interface_vals_(i);
        q_dot_kdl_(i) = velocity_interface_vals_(i);
    }

    // Read the goal pose
    xd_kdl_ = *rt_goal_pose_buffer_.readFromRT()->get();

    /**
     * TODO: implement attractive force/velocity
     */

    
    /**
     * TODO: implement joint limits as APF
     */


    /**
     * TODO: implement obstacle avoidance with APF
     */
    // Read the obstacle info
    const auto& obstacles = *rt_obstacles_.readFromRT();
    for (const auto& obs : obstacles) {
        Eigen::Vector3d p_obs(obs.position.x, obs.position.y, obs.position.z);

        // Spherical obstacle
        if (obs.shape_type == custom_msgs::msg::Obstacle::SPHERE) {
            // Sphere radius
            double obs_r = obs.radius;
            /**
             * TODO: spherical object avoidance
             */
        }
        // Box obstacle (such as wall)
        else if (obs.shape_type == custom_msgs::msg::Obstacle::BOX) {
            // Box dimensions vector
            Eigen::Vector3d box_size(obs.box_size.at(0), obs.box_size.at(1), obs.box_size.at(2));
            /**
             * TODO: box obstacle avoidance
             */
        }
    }

    /**
     * TODO: final torque command
     */

    // Send torque commands to the hardware command interface
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        command_interfaces_[i].set_value(0.0);
    }

    publish_markers();

    return controller_interface::return_type::OK;
}

CallbackReturn PotentialFieldController::on_init() {
    // Get parameters from franka_controllers.yaml
    try {
        joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
        command_interface_types_ = auto_declare<std::vector<std::string>>(
            "command_interfaces", command_interface_types_);
        state_interface_types_ = auto_declare<std::vector<std::string>>(
            "state_interfaces", state_interface_types_);

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

CallbackReturn PotentialFieldController::on_configure(
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

    // Get the joint angle limits from the URDF
    for (std::size_t i = 0; i < NUM_JOINTS; i++) {
        urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_names_[i]);
        if (!joint_urdf) {
            RCLCPP_ERROR(get_node()->get_logger(), "Could not find joint '%s' in urdf", joint_names_[i].c_str());
            return CallbackReturn::ERROR;
        }
        /**
         * TODO: read the joint limits to some data structure
         */
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
        "goal_pose", rclcpp::SystemDefaultsQoS(), std::bind(&PotentialFieldController::goal_pose_callback,
        this, std::placeholders::_1));

    init_obstacle_subscriber_ = get_node()->create_subscription<custom_msgs::msg::ObstacleArray>(
        "obstacles", rclcpp::QoS(1).transient_local(), std::bind(&PotentialFieldController::init_obstacle_callback,
        this, std::placeholders::_1));

    sub_obj_pose_ = get_node()->create_subscription<tf2_msgs::msg::TFMessage>(
        "world/obstacle_world/pose/info", rclcpp::SystemDefaultsQoS(),
        std::bind(&PotentialFieldController::object_pose_callback, this, std::placeholders::_1));

    // Publisher
    pub_markers_ = get_node()->create_publisher<visualization_msgs::msg::MarkerArray>("markers", 1000);

    RCLCPP_INFO(get_node()->get_logger(), "Robot configuration done.");
    return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
PotentialFieldController::command_interface_configuration() const {
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
PotentialFieldController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto & jname : joint_names_) {
        for (const auto & interface_type : state_interface_types_) {
            config.names.push_back(jname + "/" + interface_type);
        }
    }
    return config;
}

CallbackReturn PotentialFieldController::on_activate(
  const rclcpp_lifecycle::State& /*previous_state*/) {
    updateJointStates();
    elapsed_time_ = 0.0;

    q_kdl_.resize(NUM_JOINTS);
    q_dot_kdl_.resize(NUM_JOINTS);

    // Initialize realtime buffer
    auto xd = std::make_shared<KDL::Frame>();
    xd->p = KDL::Vector(0.5, 0.0, 1.6);
    xd->M = KDL::Rotation::EulerZYX(0.0, 0.0, M_PI);
    rt_goal_pose_buffer_.writeFromNonRT(xd);

    // Activate publisher
    pub_markers_->on_activate();

    return CallbackReturn::SUCCESS;
}

void PotentialFieldController::updateJointStates() {
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

// Joint command subscriber callback
void PotentialFieldController::goal_pose_callback(const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg) {
    auto xd = std::make_shared<KDL::Frame>();

    xd->p = KDL::Vector(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    xd->M = KDL::Rotation::Quaternion(
        msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);

    rt_goal_pose_buffer_.writeFromNonRT(xd);
}

void PotentialFieldController::init_obstacle_callback(const custom_msgs::msg::ObstacleArray msg) {
    obstacle_map_.clear(); // In case a new scene was loaded, otherwise no new messages will be published
    std::vector<custom_msgs::msg::Obstacle> obs_vec;
    for (auto obs : msg.obstacles) {
        obstacle_map_.emplace(obs.name, obs);
        obs_vec.push_back(obs);
    }

    rt_obstacles_.writeFromNonRT(obs_vec);
}

void PotentialFieldController::object_pose_callback(const tf2_msgs::msg::TFMessage& msg) {
    for (geometry_msgs::msg::TransformStamped t : msg.transforms) {
        auto it = obstacle_map_.find(t.child_frame_id);
        if (it != obstacle_map_.end()) {
            it->second.position.x = t.transform.translation.x;
            it->second.position.y = t.transform.translation.y;
            it->second.position.z = t.transform.translation.z;
            it->second.orientation = t.transform.rotation;
        }
    }

    // Build a fresh snapshot vector to write to rt buffer
    std::vector<custom_msgs::msg::Obstacle> snapshot;
    snapshot.reserve(obstacle_map_.size());
    for (const auto& [name, obs] : obstacle_map_) {
        snapshot.push_back(obs);
    }

    rt_obstacles_.writeFromNonRT(snapshot);
}

void PotentialFieldController::publish_markers() {
    visualization_msgs::msg::MarkerArray msg;

    const auto& obstacles = *rt_obstacles_.readFromRT();
    int i = 0;
    for (const auto& obs : obstacles){
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
        }

        else if (obs.shape_type == custom_msgs::msg::Obstacle::BOX) {
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.scale.x = obs.box_size.at(0);
            marker.scale.y = obs.box_size.at(1);
            marker.scale.z = obs.box_size.at(2);
        }

        // Push back to marker array
        msg.markers.push_back(marker);
    }

    pub_markers_->publish(msg);
}

// Debug print functions to reduce clutter in controller code
void PotentialFieldController::kdl_chain_error_print() {
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

void PotentialFieldController::kdl_chain_success_print() {
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

}  // namespace ex6
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ex6::PotentialFieldController,
                       controller_interface::ControllerInterface)