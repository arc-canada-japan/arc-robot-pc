import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import ExecuteProcess, LogInfo
import yaml

CURRENT_PACKAGE = os.path.basename(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def generate_launch_description():
    args = LaunchConfiguration('args')

    script_path = PathJoinSubstitution([
        get_package_share_directory(CURRENT_PACKAGE),
        CURRENT_PACKAGE,
        'nep_server.py'
    ])

    config_file_path = args.perform(None)  # Get the path to the YAML file TODO: Check if this is correct
    with open(config_file_path, 'r') as f:
        params = yaml.safe_load(f)

    return LaunchDescription([
        LogInfo(msg=['Launching nep_server.py with parameters: param1=', params['/**']['ros__parameters']]),
        LogInfo(msg=['Using package: ', CURRENT_PACKAGE]),
        LogInfo(msg=['Script path: ', script_path]),

        # Run the Python script with parameters
        ExecuteProcess(
            cmd=['python3', str(script_path), "--args", args],
            output='screen'
        )
    ])