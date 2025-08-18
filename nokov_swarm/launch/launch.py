from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nokov_swarm',
            executable='nokov_swarm_node',
            name='nokov_swarm_node',
            output='screen',
            arguments=['--ros-args', '--log-level', 'info'],
        )
    ])