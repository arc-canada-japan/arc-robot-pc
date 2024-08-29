import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import ExecuteProcess, LogInfo
import yaml

CURRENT_PACKAGE = os.path.basename(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def generate_launch_description():
    return LaunchDescription([
            LogInfo(msg=['Using remote ZMQ server']),
            LogInfo(msg=['Using package: ', CURRENT_PACKAGE]),
    ])