import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource

"""
    This generic launcher calls automatically the good controller with the corresponding config
    files according to the argument 'robot_name'.

    This launcher will call the minimal launcher (tcp server and video streaming) in addition
    to the robot controller.

    In order to work, the name of the config file should be "robot_name.yaml" and the controller
    "robot_name_controller.py".
"""

CURRENT_PACKAGE = os.path.basename(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def launch_setup(context):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    streaming_method = LaunchConfiguration('streaming_method').perform(context)
    communication_interface = LaunchConfiguration('communication_interface').perform(context)
    simulation_only = LaunchConfiguration('simulation_only').perform(context)

    launch_dir = os.path.join(get_package_share_directory(CURRENT_PACKAGE), 'launch')

    # Include minimal.launch.py
    minimal_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(launch_dir, 'minimal.launch.py')]),
        launch_arguments={
            'streaming_method': streaming_method,
            'communication_interface': communication_interface
        }.items()
    )

    # Check if the robot-specific launch file exists (not all robots have one)
    robot_launch_file = os.path.join(launch_dir, f"{robot_name}.launch.py")
    robot_launch = None
    if os.path.exists(robot_launch_file):
        robot_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_launch_file])
        )

    # Define the configuration file path
    config = os.path.join(
        get_package_share_directory(CURRENT_PACKAGE),
        'config',
        robot_name + '.yaml'
    )

    # Define the controller node
    node = Node(
        package=CURRENT_PACKAGE,
        executable=robot_name + '_controller',
        name=robot_name + '_controller',
        output='screen',
        parameters=[
            config,
            {'communication_interface': communication_interface, 
             'simulation_only': simulation_only == "True"}
        ]
    )

    # Return the components
    actions = [minimal_launch, node]
    if robot_launch:
        actions.append(robot_launch)
    return actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='NO_NAME', description='Name of the robot'),
        DeclareLaunchArgument('streaming_method', default_value='NO_METHOD', description='The method to stream the video'),
        DeclareLaunchArgument('communication_interface', default_value='NO_METHOD', description='The method to communicate between the two PCs'),
        DeclareLaunchArgument('simulation_only', default_value="False", description='If true, the robot will not be controlled (but the computed joints positions will still be published)'),
        GroupAction(
            actions=[
                PushRosNamespace('ARC'),
                OpaqueFunction(function=launch_setup) # required in order to read the value of the argument inside this launch file (to use it in file name)
            ]
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
