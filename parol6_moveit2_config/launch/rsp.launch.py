from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():
    # Robot Description
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("parol6_description"),
            "urdf",
            "parol6.urdf.xacro"
        ])
    ])
    robot_description = {"robot_description": robot_description_content}

    # Robot State Publisher node
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": True}],
        output="screen"
    )

    # ros2_control controller manager node
    # controller_manager = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[
    #         robot_description,
    #         PathJoinSubstitution([
    #             FindPackageShare("parol6_moveit2_config"),
    #             "config",
    #             "ros2_controllers.yaml"
    #         ])
    #     ],
    #     output="screen"
    # )

    return LaunchDescription([
        rsp_node,
        # controller_manager
    ])
