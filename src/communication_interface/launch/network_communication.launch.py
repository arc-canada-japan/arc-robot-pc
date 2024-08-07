import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

CURRENT_PACKAGE = os.path.basename(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def launch_setup(context):
    method = LaunchConfiguration('communication_interface').perform(context)
    config = os.path.join(
        get_package_share_directory(CURRENT_PACKAGE),
        'config',
        method + '.yaml'
    )

    launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory(CURRENT_PACKAGE), 'launch'),
         '/'+method+'.launch.py']),
         launch_arguments={'args': config}.items()
    )

    return [launch]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('communication_interface', default_value='NO_METHOD', description='The method to communicate between the two PCs'),
        GroupAction(
            actions=[
                #PushRosNamespace('ARC'),
                OpaqueFunction(function=launch_setup) # required in order to read the value of the argument inside this launch file (to use it in file name)
            ]
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
