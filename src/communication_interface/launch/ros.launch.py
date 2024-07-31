import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

CURRENT_PACKAGE = os.path.basename(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def generate_launch_description():
    args = LaunchConfiguration('args')

    tcp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory(CURRENT_PACKAGE), 'launch'),
         '/tcp_server.launch.py']),
         launch_arguments={'args': args}.items() 
    )

    return LaunchDescription([
        tcp
    ])