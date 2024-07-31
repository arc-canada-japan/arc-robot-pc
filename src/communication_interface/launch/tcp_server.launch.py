import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

CURRENT_PACKAGE = os.path.basename(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def generate_launch_description():
    param = LaunchConfiguration('args')
    return LaunchDescription([
        # Server to receive the ROS command from Unity
        Node(
            package='ros_tcp_endpoint',
            executable='default_server_endpoint',
            name='server_endpoint',
            output='screen',
            arguments=['--wait'],
            respawn=True,
            parameters=[param]
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
