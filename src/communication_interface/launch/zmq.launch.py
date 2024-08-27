import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import ExecuteProcess, LogInfo
import yaml

CURRENT_PACKAGE = os.path.basename(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def generate_launch_description():
    args = LaunchConfiguration('args')


    config_file_path = args.perform(None)  # Get the path to the YAML file TODO: Check if this is correct
    with open(config_file_path, 'r') as f:
        params = yaml.safe_load(f)

        return LaunchDescription([ # TODO: check ow to connect to the remote server
            LogInfo(msg=['Using remote ZMQ server with parameters: param1=', params['/**']['ros__parameters']]),
            LogInfo(msg=['Using package: ', CURRENT_PACKAGE]),
        ])