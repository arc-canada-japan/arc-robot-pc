import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

CURRENT_PACKAGE = os.path.basename(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ffmpeg_ip', default_value='192.168.11.50'),
        DeclareLaunchArgument('ffmpeg_port', default_value='8080'),
        DeclareLaunchArgument('video_x', default_value='1280'),
        DeclareLaunchArgument('video_y', default_value='720'),
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),

        # Call of ffmpeg
        Node(
            package='robot_control',
            executable='ffmpeg_caller',
            name='ffmpeg_caller',
            output='screen',
            parameters=[
                {'ffmpeg_ip': LaunchConfiguration('ffmpeg_ip')},
                {'ffmpeg_port': LaunchConfiguration('ffmpeg_port')},
                {'video_x': LaunchConfiguration('video_x')},
                {'video_y': LaunchConfiguration('video_y')},
                {'video_device': LaunchConfiguration('video_device')}
            ]
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
