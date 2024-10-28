from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('ai_robot')
    
    # Create path to rviz config file
    rviz_config_path = os.path.join(pkg_share, 'config', 'config.rviz')

    return LaunchDescription([
        Node(
            package='ai_robot',
            executable='ros_odom',
            name='ros_odom',
            output='screen',
        ),
        Node(
            package='ai_robot',
            executable='ros_teleop',
            name='ros_teleop',
            output='screen',
        ),
        Node(
            package='ai_robot',
            executable='ros_scan',
            name='ros_scan',
            output='screen',
        ),
        Node(
            package='ai_robot',
            executable='ros_ocgm',
            name='ros_ocgm',
            # output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),
        Node(
            package='ai_robot',
            executable='ros_joy',
            name='ros_joy',
            output='screen',
        ),
    ])