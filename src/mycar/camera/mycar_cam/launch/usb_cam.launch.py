from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()
    usb_cam_dir = get_package_share_directory('mycar_cam')

    params_path = os.path.join(
        usb_cam_dir,
        'params',
        'params.yaml'
    )

    ld.add_action(Node(
        package='usb_cam', executable='usb_cam_node_exe', output='screen',
        name="cam",
        # namespace="usb_cam",
        parameters=[params_path]
        ))
    return ld