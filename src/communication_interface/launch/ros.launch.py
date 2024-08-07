import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CURRENT_PACKAGE = os.path.basename(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def generate_launch_description():
    param = LaunchConfiguration('args')

    tcp = Node(
            package='ros_tcp_endpoint',
            executable='default_server_endpoint',
            name='server_endpoint',
            output='screen',
            arguments=['--wait'],
            respawn=True,
            parameters=[param]
        )

    return LaunchDescription([
        tcp
    ])

if __name__ == '__main__':
    generate_launch_description()