import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

CURRENT_PACKAGE = os.path.basename(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
ROBOT_NAME = os.path.basename(__file__).split('.')[0]

def generate_launch_description():
    minimal_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory(CURRENT_PACKAGE), 'launch'),
         '/minimal.launch.py'])
    )

    config = os.path.join(
      get_package_share_directory(CURRENT_PACKAGE),
      'config',
      ROBOT_NAME + '.yaml'
      )

    return LaunchDescription([
        minimal_launch,       

        # Node to command the xArm
        Node(
            package=CURRENT_PACKAGE,
            executable=ROBOT_NAME + '_controller',
            name=ROBOT_NAME + '_controller',
            output='screen',
            parameters=[config]
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
