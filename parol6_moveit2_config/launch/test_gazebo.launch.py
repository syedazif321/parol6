#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, SetEnvironmentVariable, IncludeLaunchDescription
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    full_path = os.path.join(package_path, file_path)
    try:
        with open(full_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    # Arguments
    use_sim = LaunchConfiguration("use_sim")
    declare_use_sim = DeclareLaunchArgument(
        "use_sim", default_value="true",
        description="Use simulation clock"
    )

    # Paths
    pkg_desc = get_package_share_directory("parol6_description")
    pkg_moveit = get_package_share_directory("parol6_moveit2_config")
    urdf_path = os.path.join(pkg_desc, "urdf", "parol6.urdf.xacro")

    # Generate URDF from xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        urdf_path
    ])
    robot_description = {"robot_description": robot_description_content}

    # SRDF
    srdf_path = os.path.join(pkg_moveit, "config", "parol6.srdf")
    with open(srdf_path, "r") as f:
        robot_description_semantic_config = f.read()
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_config}

    # MoveIt configs
    kinematics_yaml = load_yaml("parol6_moveit2_config", "config/kinematics.yaml")
    joint_limits_yaml = load_yaml("parol6_moveit2_config", "config/joint_limits.yaml")
    joint_limits = {"robot_description_planning": joint_limits_yaml} if joint_limits_yaml else {}
    servo_yaml = load_yaml("parol6_moveit2_config", "config/moveit_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml} if servo_yaml else {}

    # Set Gazebo model path so meshes are found
    set_gazebo_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=[
            TextSubstitution(text=os.path.join(pkg_desc, "meshes")),
            TextSubstitution(text=":" + os.environ.get("GAZEBO_MODEL_PATH", ""))
        ]
    )

    # Gazebo launch
    world_file_path = os.path.join(
        os.path.dirname(pkg_desc), "parol6_gazebo", "worlds", "table1.world"
    )
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"
            ])
        ]),
        launch_arguments={"world": world_file_path, "use_sim_time": use_sim}.items()
    )

    # Robot State Publisher
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim}]
    )

    # Spawn entity in Gazebo after RSP starts
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "parol6"],
        output="screen"
    )
    spawn_after_rsp = RegisterEventHandler(
        OnProcessStart(target_action=rsp_node, on_start=[spawn_entity])
    )

    # Controllers
    spawner_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )
    spawner_arm = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    # Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            {"robot_description_kinematics": kinematics_yaml},
            joint_limits,
            {"use_sim_time": use_sim}
        ]
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d", PathJoinSubstitution([
                FindPackageShare("parol6_moveit2_config"), "config", "moveit.rviz"
            ])
        ],
        parameters=[
            robot_description,
            robot_description_semantic,
            {"robot_description_kinematics": kinematics_yaml},
            joint_limits,
            {"use_sim_time": use_sim}
        ]
    )

    # Servo Node
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
            {"robot_description_kinematics": kinematics_yaml},
            joint_limits,
            {"use_sim_time": use_sim}
        ],
        remappings=[
            ("~/joint_states", "/joint_states"),
            ("~/robot_description", "/robot_description")
        ]
    )

    return LaunchDescription([
        declare_use_sim,
        set_gazebo_model_path,
        gazebo_launch,
        rsp_node,
        spawn_after_rsp,
        spawner_jsb,
        spawner_arm,
        move_group_node,
        rviz_node,
        servo_node
    ])
