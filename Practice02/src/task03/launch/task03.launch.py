from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('task03')
    params_file = os.path.join(pkg_share, 'config', 'task03.yaml')

    service_name_arg = DeclareLaunchArgument(
        'service_name',
        default_value='/trigger_service',
        description='Name of the Trigger service provided by the node'
    )

    return LaunchDescription([
        service_name_arg,
        Node(
            package='task03',
            executable='trigger_proxy',
            name='trigger_proxy',
            output='screen',
            parameters=[
                params_file,
                {'service_name': LaunchConfiguration('service_name')}
            ],
        )
    ])
