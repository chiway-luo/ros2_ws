from launch import LaunchDescription
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    return LaunchDescription([
        # map_file,
        launch_ros.actions.Node(
          package='nav2_map_server',
          executable='map_saver_cli',
          name='map_saver_cli',
          output='screen',
          arguments=['-f', 'map/nav']
        )
    ])