from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to your node's package and launch file
    communication_interface_dir = get_package_share_directory('communication_interface')
    network_communication_launch = os.path.join(communication_interface_dir, 'launch', 'network_communication.launch.py')

    # Declare the launch argument for enabling the fake publisher
    fake_pub_arg = DeclareLaunchArgument(
        'fake_pub',
        default_value='false',
        description='Set to true to enable the fake publisher node'
    )

    return LaunchDescription([
        # Launch argument
        fake_pub_arg,

        # Include the external launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(network_communication_launch),
            launch_arguments={'communication_interface': 'ros'}.items()
        ),

        # Launch your custom node
        Node(
            package='communication_interface',
            executable='ros_to_zmq_bridge',
            name='zmq_interface_node',
            output='screen',
            parameters=[{'config_file': '/home/yoshidalab/arc_ws/arc-local/src/communication_interface/config/zmq_OPERATOR.yaml'}]
        ),

        # Conditionally launch the fake publisher node
        Node(
            package='communication_interface',
            executable='end_effector_fake_publisher',
            name='end_effector_fake_publisher_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('fake_pub'))
        ),
    ])
