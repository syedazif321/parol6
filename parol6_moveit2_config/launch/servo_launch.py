#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml


def generate_launch_description():
    ld = LaunchDescription()

    use_sim = LaunchConfiguration('use_sim')
    declare_use_sim = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Start robot in Gazebo simulation.'
    )
    ld.add_action(declare_use_sim)

    # Robot Description from Xacro
    parol6_description_path = get_package_share_directory('parol6_description')
    xacro_file = os.path.join(parol6_description_path, 'urdf', 'parol6.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # SRDF
    srdf_file = os.path.join(
        get_package_share_directory('parol6_moveit2_config'),
        'config',
        'parol6.srdf'
    )
    with open(srdf_file, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    # Kinematics
    kinematics_file = os.path.join(
        get_package_share_directory('parol6_moveit2_config'),
        'config',
        'kinematics.yaml'
    )
    with open(kinematics_file, 'r') as f:
        kinematics_yaml = yaml.safe_load(f)

    # MoveIt Servo config
    servo_file = os.path.join(
        get_package_share_directory('parol6_moveit2_config'),
        'config',
        'moveit_servo.yaml'
    )
    with open(servo_file, 'r') as f:
        servo_params = {'moveit_servo': yaml.safe_load(f)}

    # Launch MoveIt Servo node
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        parameters=[
            {'use_gazebo': use_sim},
            servo_params,
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
        ],
        output='screen'
    )
    ld.add_action(servo_node)

    return ld
