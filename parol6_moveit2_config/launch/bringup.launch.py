from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, TextSubstitution
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("parol6", package_name="parol6_moveit2_config")
        .robot_description(file_path="config/parol6.urdf.xacro")
        .robot_description_semantic(file_path="config/parol6.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"
            ])
        ]),
        launch_arguments={
            'world': TextSubstitution(
                text='/home/azif/projetcs/parol6/parol6_gazebo/worlds/table1.world'
            ),
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
