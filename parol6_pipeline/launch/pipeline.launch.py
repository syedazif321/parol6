from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

def generate_launch_description():
    # Start Gazebo (or your robot sim)
    # gazebo = ExecuteProcess(
    #     cmd=['ros2', 'launch', 'parol6_gazebo', 'gazebo.launch.py'],
    #     output='screen',
    #     shell=True
    # )

    # Start box detector node
    box_detector_node = Node(
        package='parol6_pipeline',
        executable='box_detector_node',
        name='box_detector_node',
        output='screen'
    )

    # Start pick controller
    pick_controller = Node(
        package='parol6_pipeline',
        executable='pick_controller',
        name='pick_controller',
        output='screen'
    )

    # Start pipeline node (orchestrator) - starts last
    pipeline_node = Node(
        package='parol6_pipeline',
        executable='pipeline_node',
        name='pipeline_node',
        output='screen'
    )

    # Sequence: Gazebo → box_detector → pick_controller → pipeline
    return LaunchDescription([
        # 1. Start Gazebo
        gazebo,

        # 2. After Gazebo starts, launch box_detector_node
        RegisterEventHandler(
            OnProcessStart(
                target_action=gazebo,
                on_start=[
                    TimerAction(
                        period=3.0,  # Wait 3 sec for Gazebo to boot
                        actions=[box_detector_node]
                    )
                ]
            )
        ),

        # 3. After box_detector starts, launch pick_controller
        RegisterEventHandler(
            OnProcessStart(
                target_action=box_detector_node,
                on_start=[
                    TimerAction(
                        period=2.0,
                        actions=[pick_controller]
                    )
                ]
            )
        ),

        # 4. After pick_controller starts, launch pipeline
        RegisterEventHandler(
            OnProcessStart(
                target_action=pick_controller,
                on_start=[
                    TimerAction(
                        period=2.0,
                        actions=[pipeline_node]
                    )
                ]
            )
        ),
    ])