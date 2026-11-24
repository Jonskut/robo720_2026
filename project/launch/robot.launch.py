import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Tell Gazebo where to find our custom models (conveyor, heads, obstacle)
    models_dir = PathJoinSubstitution([
        FindPackageShare("project"),
        "model"
    ])
    set_model_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[models_dir, ":", EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value="")]
    )

    # Controller configuration
    initial_joint_controllers = PathJoinSubstitution([
        FindPackageShare("project"),
        "config",
        "franka_controllers.yaml"
    ])

    # World file
    world_file = os.path.join(
        get_package_share_directory("project"), "world", "project_world.sdf"
    )

    # Robot description from xacro (FT sensor enabled; vision uses fixed side camera)
    robot_description_xacro = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("franka_description"),
            "urdf",
            "effort_fr3_arm.urdf.xacro"
        ]),
        " ",
        "sim_ignition:=true",
        " ",
        "simulation_controllers:=", initial_joint_controllers,
        " ",
        "effort_commands:=true",
        " ",
        "camera:=false",
        " ",
        "end_effector:=ft_sensor",
    ])

    # Nodes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True,
                     "robot_description": ParameterValue(robot_description_xacro, value_type=str)}],
        remappings=[("robot_description", "robot_description")]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    project_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["project_controller", "--controller-manager", "/controller_manager"],
    )

    # Static TF so frame "map" exists (RViz default fixed frame)
    static_tf_map_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "map", "base"],
    )

    # GZ nodes
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string", robot_description_xacro,
            "-name", "fr3_arm",
            "-allow_renaming", "true",
        ],
    )

    gz_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"
        ]),
        launch_arguments={"gz_args": f" -r -v 1 {world_file}"}.items(),
    )

    # RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=[]
    )

    # Bridge Gazebo topics to ROS
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="pose_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/side_camera/image@sensor_msgs/msg/Image@ignition.msgs.Image",
            "/side_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
            "/sensor_joint/force_torque@geometry_msgs/msg/WrenchStamped@ignition.msgs.Wrench",
            "/world/project_world/pose/info@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V",
        ],
    )

    # Fake wrench publisher (ex7 parity) -> enables compliance feedback via
    # 'ros2 param set /project_controller use_fake_wrench true'
    fake_wrench_publisher = Node(
        package="goal_publishers",
        executable="fake_wrench_publisher.py",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # Static obstacle description (latched) for potential-field avoidance
    obstacle_publisher = Node(
        package="project",
        executable="obstacle_publisher.py",
        output="screen",
    )

    # Conveyor animation: slides the heads (and their markers) back and forth
    conveyor_node = Node(
        package="project",
        executable="conveyor.py",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # ArUco marker pose estimation (visual servoing input) on the fixed side camera
    pose_estimate = Node(
        package="project",
        executable="pose_estimate.py",
        output="screen",
        arguments=[
            "--ros-args",
            "-p", "image_topic:=/side_camera/image",
            "-p", "camera_info_topic:=/side_camera/camera_info",
            "-p", "dictionary:=DICT_4X4_50",
            "-p", "marker_id:=0",
            "-p", "marker_length_m:=0.1026",
        ],
    )

    nodes_to_start = [
        set_model_path,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        project_controller_spawner,
        static_tf_map_base,
        gz_spawn_entity,
        gz_launch_description,
        rviz_node,
        ros_gz_bridge,
        fake_wrench_publisher,
        obstacle_publisher,
        conveyor_node,
        pose_estimate,
        SetEnvironmentVariable('IGN_GAZEBO_VERBOSE', '1'),
    ]

    return LaunchDescription(nodes_to_start)
