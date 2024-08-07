import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

CURRENT_PACKAGE = os.path.basename(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def generate_launch_description():
    # Declare the 'streaming_method' argument
    streaming_method_arg = DeclareLaunchArgument(
        'streaming_method',
        default_value='ffmpeg',
        description='Method to use for streaming video'
    )

    communication_interface_arg = DeclareLaunchArgument(
        'communication_interface',
        default_value='ros',
        description='The method to communicate between the two PCs'
    )

    video_stream = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('video_publisher'),
                    'launch',
                    'video_stream.launch.py'
                ])
            ]),
            launch_arguments={
                'streaming_method': LaunchConfiguration('streaming_method')
            }.items()
        )

    communication = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('communication_interface'),
                    'launch',
                    'network_communication.launch.py'
                ])
            ]),
            launch_arguments={
                'communication_interface': LaunchConfiguration('communication_interface')
            }.items()
        )


    return LaunchDescription([
        streaming_method_arg,
        communication_interface_arg,
        communication,
        video_stream
    ])

if __name__ == '__main__':
    generate_launch_description()
