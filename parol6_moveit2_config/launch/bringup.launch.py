#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

import os
import xacro
import yaml


def load_yaml(package_name, file_path):
    """Load YAML file from package."""
    package_path = get_package_share_directory(package_name)
    full_path = os.path.join(package_path, file_path)
    try:
        with open(full_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError as e:
        print(f"Error loading YAML: {e}")
        return None


def generate_launch_description():
    use_sim = LaunchConfiguration('use_sim')
    declare_use_sim = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    pkg_share_dir = get_package_share_directory('parol6_moveit2_config')
    description_share_dir = get_package_share_directory('parol6_description')

    urdf_path = os.path.join(description_share_dir, "urdf", "parol6.urdf.xacro")
    robot_description_config = xacro.process_file(urdf_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    srdf_path = os.path.join(pkg_share_dir, "config", "parol6.srdf")
    with open(srdf_path, 'r') as f:
        robot_description_semantic_config = f.read()
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}

    kinematics_yaml = load_yaml("parol6_moveit2_config", "config/kinematics.yaml")
    if kinematics_yaml is None:
        return LaunchDescription([])

    joint_limits_yaml = load_yaml("parol6_moveit2_config", "config/joint_limits.yaml")
    joint_limits = {'robot_description_planning': joint_limits_yaml} if joint_limits_yaml else {}

    servo_yaml = load_yaml("parol6_moveit2_config", "config/moveit_servo.yaml")
    if servo_yaml is None:
        return LaunchDescription([])
    servo_params = {'moveit_servo': servo_yaml}


    launch_file_dir = os.path.dirname(os.path.realpath(__file__))
    project_root = os.path.abspath(os.path.join(launch_file_dir, "..", ".."))
    world_file_path = os.path.join(project_root, "parol6_gazebo", "worlds", "table1.world")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"
            ])
        ]),
        launch_arguments={
            'world': world_file_path,
            'use_sim_time': use_sim,
        }.items()
    )

    robot_state_pub = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("parol6_moveit2_config"), "launch", "rsp.launch.py"
            ])
        ]),
        launch_arguments={'use_sim_time': use_sim}.items()
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "parol6"],
        output="screen"
    )

    spawn_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("parol6_moveit2_config"), "launch", "spawn_controllers.launch.py"
            ])
        ]),
        launch_arguments={'use_sim_time': use_sim}.items()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            MoveItConfigsBuilder("parol6", package_name="parol6_moveit2_config")
            .robot_description(file_path=urdf_path)
            .robot_description_semantic(file_path=srdf_path)
            .trajectory_execution(file_path=os.path.join(pkg_share_dir, "config", "moveit_controllers.yaml"))
            .planning_pipelines(pipelines=["ompl"])
            .to_moveit_configs()
            .to_dict(),
            {"use_sim_time": use_sim},
            {"planning_scene_monitor.publish_planning_scene": True},
            {"planning_scene_monitor.publish_geometry_updates": True},
            {"planning_scene_monitor.publish_state_updates": True},
            {"planning_scene_monitor.publish_transforms_updates": True},
        ],
        arguments=["--ros-args", "--log-level", "move_group:=debug"], 
    )




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
            {"use_sim_time": use_sim},
        ],
        remappings=[
            ("~/joint_states", "/joint_states"),
            ("~/robot_description", "/robot_description"),
        ],
    )

    static_tf_camera_mount = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_camera_mount',
        arguments=[
            '0.071927', '-1.097241', '1.312280',
            '-1.57',  '0',  '3.14',
            'world', 'realsense_rgb_frame'  
        ]
    )

    pick_controller_node = Node(
        package="parol6_pipeline",
        executable="pick_controller_node",
        name="pick_controller",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            {"robot_description_kinematics": kinematics_yaml},
            joint_limits,
            {"use_sim_time": use_sim} 
        ],
        remappings=[

        ]
    )

    return LaunchDescription([
        declare_use_sim,
        gazebo_launch,
        robot_state_pub,
        spawn_entity,
        spawn_controllers,
        move_group_node,
        rviz_node,
        servo_node,
        static_tf_camera_mount,
        pick_controller_node,

    ])
