import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

CURRENT_PACKAGE = os.path.basename(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('tcp_ip', default_value='0.0.0.0'),
        DeclareLaunchArgument('tcp_port', default_value='10000'),

        # Server to receive the ROS command from Unity
        Node(
            package='ros_tcp_endpoint',
            executable='default_server_endpoint',
            name='server_endpoint',
            output='screen',
            arguments=['--wait'],
            respawn=True,
            parameters=[
                {'tcp_ip': LaunchConfiguration('tcp_ip')},
                {'tcp_port': LaunchConfiguration('tcp_port')}
            ]
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
