#!/usr/bin/env python3

import os
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Use sim time argument
    use_sim = LaunchConfiguration('use_sim')
    declare_use_sim = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Start robot in simulation (e.g., Gazebo)'
    )
    ld.add_action(declare_use_sim)

    # Robot description (URDF)
    xacro_file = os.path.join(
        get_package_share_directory('parol6_description'),
        'urdf',
        'parol6.urdf.xacro'
    )
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Semantic robot description (SRDF)
    srdf_path = os.path.join(
        get_package_share_directory('parol6_moveit2_config'),
        'config',
        'parol6.srdf'
    )
    with open(srdf_path, 'r') as file:
        srdf_content = file.read()
    robot_description_semantic = {'robot_description_semantic': srdf_content}

    # Kinematics config
    kinematics_path = os.path.join(
        get_package_share_directory('parol6_moveit2_config'),
        'config',
        'kinematics.yaml'
    )
    with open(kinematics_path, 'r') as file:
        kinematics_yaml = yaml.safe_load(file)

    # Servo config
    servo_yaml_path = os.path.join(
        get_package_share_directory('parol6_moveit2_config'),
        'config',
        'moveit_servo.yaml'
    )
    with open(servo_yaml_path, 'r') as file:
        servo_params = {'moveit_servo': yaml.safe_load(file)}

    # Servo Node
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim},
            servo_params,
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
        ]
    )
    ld.add_action(servo_node)

    return ld
