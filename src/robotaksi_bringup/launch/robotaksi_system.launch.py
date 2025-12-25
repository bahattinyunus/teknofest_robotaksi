from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Perception Node
        Node(
            package='robotaksi_perception',
            executable='obstacle_detector',
            name='perception_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        # Planning Node
        Node(
            package='robotaksi_planning',
            executable='global_planner',
            name='planning_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        # Control Node
        Node(
            package='robotaksi_control',
            executable='pid_controller',
            name='control_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
    ])
