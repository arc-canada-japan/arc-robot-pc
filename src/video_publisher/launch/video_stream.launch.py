import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

CURRENT_PACKAGE = os.path.basename(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def launch_setup(context):
    streaming_method = LaunchConfiguration('streaming_method').perform(context)

    config = os.path.join(
        get_package_share_directory(CURRENT_PACKAGE),
        'config',
        streaming_method + '.yaml'
    )

    node = Node(
        package=CURRENT_PACKAGE,
        executable=streaming_method + '_streamer',
        name=streaming_method + '_streamer',
        output='screen',
        parameters=[config]
    )

    return [node]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('streaming_method', default_value='NO_METHOD', description='The method to stream the video'),
        OpaqueFunction(function=launch_setup) # required in order to read the value of the argument inside this launch file (to use it in file name)
    ])

if __name__ == '__main__':
    generate_launch_description()
