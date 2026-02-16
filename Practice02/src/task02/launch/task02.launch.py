from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('task02')
    params_file = os.path.join(pkg_share, 'config', 'task02.yaml')

    text_arg = DeclareLaunchArgument(
        'text',
        default_value='Hello, ROS2!',
        description='Text to publish'
    )

    return LaunchDescription([
        text_arg,
        Node(
            package='task02',
            executable='publisher',
            name='publisher',
            output='screen',
            parameters=[
                params_file,  # provides topic_name
                {'text': LaunchConfiguration('text')}  # override via launch arg
            ],
        )
    ])
