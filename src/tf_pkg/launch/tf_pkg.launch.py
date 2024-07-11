from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='tf_pkg',
            executable='publisher',
            name='my_publish',
            parameters=[
                {'turtlename': 'turtle1'}
            ]
        ),
        DeclareLaunchArgument(
            'target_frame', default_value='turtle1',
            description='Target frame name.'
        ),
        Node(
            package='tf_pkg',
            executable='publisher',
            name='my_publish2',
            parameters=[
                {'turtlename': 'turtle2'}
            ]
        ),
    ])