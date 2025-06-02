from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Member 1: Flight Control Nodes
        Node(
            package='drone_control_pkg',
            executable='flight_controller',
            name='flight_controller',
            output='screen'
        ),
        Node(
            package='drone_control_pkg',
            executable='safety_monitor',
            name='safety_monitor', 
            output='screen'
        ),
        
        # Member 2: Navigation Nodes
        Node(
            package='drone_control_pkg',
            executable='waypoint_navigator',
            name='waypoint_navigator',
            output='screen'
        ),
        Node(
            package='drone_control_pkg',
            executable='path_planner',
            name='path_planner',
            output='screen'
        ),
        
        # Member 3: Communication Nodes
        Node(
            package='drone_control_pkg',
            executable='mavlink_bridge',
            name='mavlink_bridge',
            output='screen'
        ),
        Node(
            package='drone_control_pkg',
            executable='telemetry_node',
            name='telemetry_node',
            output='screen'
        ),
    ])

