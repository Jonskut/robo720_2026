import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Initialize controller arguments
    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare("ex7"), "config", "franka_controllers.yaml"]
    )

    # World file
    world_file = os.path.join(
        get_package_share_directory("ex7"), "world", "force_world.sdf"
    )

    # RViz config file
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("ex7"), "config", "config.rviz"]
    )

    # Find the robot description file
    robot_description_xacro = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("franka_description"),
                 "urdf",
                 "effort_fr3_arm.urdf.xacro"]
            ),
            " ",
            "sim_ignition:=true",
            " ",
            "simulation_controllers:=",
            initial_joint_controllers,
            " ",
            "effort_commands:=true",
            " ",
            "end_effector:=ft_sensor",
        ]
    )

    # Nodes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True,
                    "robot_description": ParameterValue(robot_description_xacro, value_type=str)
        }],
        remappings=[("robot_description", "robot_description")]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    force_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["direct_force_controller", "--controller-manager", "/controller_manager"],
    )

    # GZ nodes
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_xacro,
            "-name",
            "fr3_arm",
            "-allow_renaming",
            "true",
        ],
    )

    # launch arguments for Gazebo {args, world (- v 1 = log level)}
    gz_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": f" -r -v 1 {world_file}"}.items(),
    )

    # RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=[
            '-d',
            rviz_config
        ]
    )

    # Plotjuggler node
    plotjuggler_node = Node(
        package="plotjuggler",
        executable="plotjuggler",
        output="screen",
        arguments=[
        ]
    )

    # Bridge Gazebo topics to ROS
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",  # bridge Gazebo simulation time to ROS
            "/sensor_joint/force_torque@geometry_msgs/msg/WrenchStamped@ignition.msgs.Wrench",
        ],
    )

    nodes_to_start = [
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        force_controller_spawner,
        gz_spawn_entity,
        gz_launch_description,
        rviz_node,
        plotjuggler_node,
        ros_gz_bridge,
        SetEnvironmentVariable('IGN_GAZEBO_VERBOSE', '1'),
    ]

    return LaunchDescription(nodes_to_start)