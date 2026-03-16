from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nidar_drone',
            executable='publisher_node',
            name='altitude_publisher',
            output='screen'
        ),

        Node(
            package='nidar_drone',
            executable='subscriber_node',
            name='altitude_subscriber',
            output='screen'
        ),

        Node(
            package='nidar_drone',
            executable='arm_service',
            name='arm_drone_service',
            output='screen'
        ),
        Node(
            package='nidar_drone',
            executable='arm_client',
            name='arm_drone_client',
            output='screen'
        ),

    ])