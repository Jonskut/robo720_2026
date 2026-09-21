#include "goal_publishers/trajectory_publisher.hpp"
#include "franka_kdl/solver.hpp"
#include "franka_kdl/robot_constants.hpp"

#include <cmath>
#include <cstddef>
#include <exception>

#include <rclcpp/rclcpp.hpp>
#include <urdf/model.h>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>

using namespace std::chrono_literals;

TrajectoryPublisher::TrajectoryPublisher()
: Node("trajectory_publisher")
{
    // Declare parameters
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "Robot URDF in XML string format to construct the KDL solver.";
    this->declare_parameter("robot_description", rclcpp::ParameterType::PARAMETER_STRING, param_desc);

    param_desc.description = "Root link of the KDL Chain.";
    this->declare_parameter("root_link", rclcpp::ParameterType::PARAMETER_STRING, param_desc);

    param_desc.description = "Tip link of the KDL Chain.";
    this->declare_parameter("tip_link", rclcpp::ParameterType::PARAMETER_STRING, param_desc);

    // Get parameters
    robot_description_ = this->get_parameter("robot_description").as_string(); // passed as a launch argument
    root_link_ = this->get_parameter("root_link").as_string(); // from franka_controllers.yaml
    tip_link_ = this->get_parameter("tip_link").as_string();   // this too
    trajectory_type_ = this->declare_parameter("trajectory_type", "circle");
    
    parameter_callback_handle_ =
        this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& parameters) {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
    
                for (const auto& parameter : parameters) {
                    if (parameter.get_name() != "trajectory_type") {
                        continue;
                    }
    
                    if (parameter.get_type() !=
                        rclcpp::ParameterType::PARAMETER_STRING) {
                        result.successful = false;
                        result.reason = "trajectory_type must be a string";
                        return result;
                    }
    
                    const auto value = parameter.as_string();
    
                    if (value != "circle" &&
                        value != "line" &&
                        value != "triangle" &&
                        value != "test_error") {
                        result.successful = false;
                        result.reason =
                            "trajectory_type must be circle, line, triangle or test_error";
                        return result;
                    }
    
                    trajectory_type_ = value;
    
                    RCLCPP_INFO(
                        this->get_logger(),
                        "trajectory_type changed to '%s'",
                        trajectory_type_.c_str());
                }
    
                return result;
            });

    // Check if parameter was given
    if (robot_description_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Robot description parameter not given.");
    }

    // Parse robot_description into URDF
    urdf::Model urdf;
    if (!urdf.initString(robot_description_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse urdf file");
    }
    else {
        RCLCPP_INFO(this->get_logger(), "robot_description successfully parsed into URDF");
    }

    // Construct a KDL tree object from the URDF
    if (!kdl_parser::treeFromUrdfModel(urdf, tree_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to construct a KDL tree.");
    }
    else {
        RCLCPP_INFO(this->get_logger(), "Constructed a KDL tree.");
    }

    // Get the KDL chain from KDL tree (for KDL Solver objects)
    if (!tree_.getChain(root_link_, tip_link_, chain_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to construct a KDL chain.");
    }
    else {
        RCLCPP_INFO(this->get_logger(), "Constructed a KDL chain.");
    }

    // Create solver instance with the KDL Chain
    solver_.reset(new Solver(chain_));

    // Initialize KDL variables
    q_.resize(NUM_JOINTS);
    q_cmd_.resize(NUM_JOINTS);
    q_dot_cmd_.resize(NUM_JOINTS);
    qd_dot_.resize(NUM_JOINTS);

    // Create subscriber
    joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, std::bind(&TrajectoryPublisher::joint_state_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Created a JointState subscriber.");

    // Create publisher
    trajectory_point_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>("trajectory_point", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&TrajectoryPublisher::timer_callback, this));
    trajectory_point_msg_.positions.resize(NUM_JOINTS);
    trajectory_point_msg_.velocities.resize(NUM_JOINTS);
    trajectory_point_msg_.effort.resize(NUM_JOINTS);
    RCLCPP_INFO(this->get_logger(), "Created a JointTrajectoryPoint publisher.");
}

void TrajectoryPublisher::line_trajectory(KDL::Vector& tgt_pos, KDL::Twist& tgt_vel, double t) {
    // Desired pose
    double pos_x = 0.5;
    double pos_y = 0.2 * cos(0.5 * t);
    double pos_z = 1.6;

    tgt_pos.x(pos_x); tgt_pos.y(pos_y); tgt_pos.z(pos_z);

    double vel_x = 0.0;
    double vel_y = -0.1 * sin(0.5 * t);
    double vel_z = 0.0;

    tgt_vel.vel.x(vel_x); tgt_vel.vel.y(vel_y); tgt_vel.vel.z(vel_z);
}

void TrajectoryPublisher::circle_trajectory(KDL::Vector& tgt_pos, KDL::Twist& tgt_vel, double t) {
    // Desired pose
    double pos_x = 0.5;
    double pos_y = 0.1 * cos(0.5 * t);
    double pos_z = 1.6 + 0.1 * sin(0.5 * t);

    tgt_pos.x(pos_x); tgt_pos.y(pos_y); tgt_pos.z(pos_z);

    double vel_x = 0.0;
    double vel_y = -0.05 * sin(0.5 * t);
    double vel_z = 0.05 * cos(0.5 * t);

    tgt_vel.vel.x(vel_x); tgt_vel.vel.y(vel_y); tgt_vel.vel.z(vel_z);
}

void TrajectoryPublisher::triangle_trajectory(
    KDL::Vector& tgt_pos, KDL::Twist& tgt_vel, double t) {
    const double angular_frequency = 0.5;
    const double period = 2.0 * M_PI / angular_frequency;
    const double segment_duration = period / 3.0;

    double local_time = std::fmod(t, period);
    if (local_time < 0.0) {
        local_time += period;
    }

    // Triangle vertices in the same workspace region as the circle.
    const KDL::Vector vertex_a(0.5, 0.0, 1.7);
    const KDL::Vector vertex_b(0.5, -0.0866, 1.55);
    const KDL::Vector vertex_c(0.5, 0.0866, 1.55);

    const KDL::Vector* start_vertex;
    const KDL::Vector* end_vertex;
    double time_in_segment;

    if (local_time < segment_duration) {
        start_vertex = &vertex_a;
        end_vertex = &vertex_b;
        time_in_segment = local_time;
    } else if (local_time < 2.0 * segment_duration) {
        start_vertex = &vertex_b;
        end_vertex = &vertex_c;
        time_in_segment = local_time - segment_duration;
    } else {
        start_vertex = &vertex_c;
        end_vertex = &vertex_a;
        time_in_segment = local_time - 2.0 * segment_duration;
    }

    const KDL::Vector edge = *end_vertex - *start_vertex;
    const double interpolation = time_in_segment / segment_duration;

    tgt_pos = *start_vertex + interpolation * edge;

    tgt_vel = KDL::Twist::Zero();
    tgt_vel.vel.x(edge.x() / segment_duration);
    tgt_vel.vel.y(edge.y() / segment_duration);
    tgt_vel.vel.z(edge.z() / segment_duration);
}

// Subscriber callback for current joint states
void TrajectoryPublisher::joint_state_callback(const sensor_msgs::msg::JointState& msg) {
    if (msg.position.size() < NUM_JOINTS) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "Received an incomplete joint state; keeping the previous command.");
        return;
    }

    // Copy joint state to the KDL variable
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        q_(i) = msg.position.at(i);
    }
    joint_state_received_ = true;
}

// Publisher for desired joint position and velocity
void TrajectoryPublisher::timer_callback() {
    // Get the current timestamp
    double t = this->get_clock()->now().seconds();

    // Create the cartesian pose
    KDL::Vector tgt_pos;
    KDL::Twist tgt_vel = KDL::Twist::Zero();
    if (trajectory_type_ == "circle") {
        circle_trajectory(tgt_pos, tgt_vel, t);
    }
    else if (trajectory_type_ == "line") {
        line_trajectory(tgt_pos, tgt_vel, t);
    }
    else if (trajectory_type_ == "triangle") {
        triangle_trajectory(tgt_pos, tgt_vel, t);
    }
    else {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "Unknown trajectory type '%s'; holding the current position.",
            trajectory_type_.c_str());

        if (!joint_state_received_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "Cannot hold position until a valid joint state has been received.");
            return;
        }

        for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
            trajectory_point_msg_.positions.at(i) = q_(i);
            trajectory_point_msg_.velocities.at(i) = qd_dot_(i);
        }
        trajectory_point_publisher_->publish(trajectory_point_msg_);
        return;
    }

    KDL::Rotation tgt_rot = KDL::Rotation::EulerZYX(M_PI_2, M_PI_2, M_PI_2);
    KDL::Frame tgt_pose(tgt_rot, tgt_pos);

    // Solve IK
    int ret = solver_->computeIK(q_, tgt_pose, q_cmd_);

    if (ret < 0) {
        RCLCPP_ERROR(this->get_logger(), "Inverse kinematics failed with error code: %d", ret);
    }

    // Solve velocity IK
    ret = solver_->computeIKvel(q_, tgt_vel, q_dot_cmd_);

    if (ret < 0) {
        RCLCPP_ERROR(this->get_logger(), "Velocity inverse kinematics failed with error code: %d", ret);
    }

    // Populate message
    for (std::size_t i = 0; i < NUM_JOINTS; i++) {
        trajectory_point_msg_.positions.at(i) = q_cmd_(i);
        trajectory_point_msg_.velocities.at(i) = q_dot_cmd_(i);
    }

    trajectory_point_publisher_->publish(trajectory_point_msg_);
}

// Main function for spinning the node
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    try {
        rclcpp::spin(std::make_shared<TrajectoryPublisher>());
    } catch (const std::exception& e) {
        fprintf(stderr, "Exception during node initialization: %s \n", e.what());
        rclcpp::shutdown();
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}