import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

CURRENT_PACKAGE = os.path.basename(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def generate_launch_description():
    #return LaunchDescription()
    # Path to the `moveit_vel_control_node.launch.py` file
    moveit_launch_file = os.path.join(
        get_package_share_directory('move_it_node'),
        'launch',
        'moveit_vel_control_node.launch.py'
    )

    # Path to the `pose_viz.launch.py` file
    vr_launch_file = os.path.join(
        get_package_share_directory('vr_manager'),
        'launch',
        'pose_viz.launch.py'
    )

    return LaunchDescription([
        # Node for `ros2 run gen3_control gen3_vel_control`
        Node(
            package='gen3_control',
            executable='gen3_vel_control',
            name='gen3_vel_control',
            output='screen',
            arguments=['--wait'],
            respawn=True,
        ),

        # Include the `moveit_vel_control_node.launch.py`
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit_launch_file)
        ),

        # Include the `pose_viz.launch.py`
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(vr_launch_file)
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
