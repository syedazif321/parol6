from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

import os

def generate_launch_description():
    # URDF path from package
    urdf_path = os.path.join(
        get_package_share_directory("parol6_description"),
        "urdf",
        "parol6.urdf.xacro"
    )

    # MoveIt config builder
    moveit_config = (
        MoveItConfigsBuilder("parol6", package_name="parol6_moveit2_config")
        .robot_description(file_path=urdf_path)
        .robot_description_semantic(file_path="config/parol6.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Use relative path for world file (assumes this file is in parol6_moveit2_config/launch/)
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
            'world': TextSubstitution(text=world_file_path),
            'use_sim_time': 'True',
        }.items()
    )

    robot_state_pub = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("parol6_moveit2_config"), "launch", "rsp.launch.py"
            ])
        ])
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
        ])
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.planning_pipelines,
            moveit_config.trajectory_execution,
            {"use_sim_time": True}
        ]
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
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": True}
        ]
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_pub,
        spawn_entity,
        spawn_controllers,
        move_group_node,
        rviz_node
    ])
