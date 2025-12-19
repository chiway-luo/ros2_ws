from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    MYCAR_MODEL = os.environ['MYCAR_MODEL']
    LIDAR = os.environ['LIDAR']
    mycar_launch_file_path = ''
    lidar_launch_file_path = ''

    if MYCAR_MODEL == 'arduino':
        mycar_launch_file_path=os.path.join(
            get_package_share_directory("ros2_arduino_bridge"),
            "launch",
            "ros2_arduino.launch.py"
        )
    else:
        mycar_launch_file_path=os.path.join(
            get_package_share_directory("ros2_stm32_bridge"),
            "launch",
            "driver.launch.py"
        )
    
    mycar_launch = IncludeLaunchDescription(
        launch_description_source= PythonLaunchDescriptionSource(
            launch_file_path = mycar_launch_file_path
        )
    )
    
    if LIDAR == "sl_A1":
        lidar_launch_file_path=os.path.join(
            get_package_share_directory("sllidar_ros2"),
            "launch",
            "sllidar_launch.py"
        )
    elif LIDAR == "ls_N10":
        lidar_launch_file_path=os.path.join(
            get_package_share_directory("lslidar_driver"),
            "launch",
            "lsn10_launch.py"
        )


    lidar_launch = IncludeLaunchDescription(
        launch_description_source= PythonLaunchDescriptionSource(
            launch_file_path=lidar_launch_file_path
        )
    )
    cam_launch = IncludeLaunchDescription(
        launch_description_source= PythonLaunchDescriptionSource(
            launch_file_path=os.path.join(
                get_package_share_directory("mycar_cam"),
                "launch",
                "usb_cam.launch.py"
            )
        )
    )
    tf_pub_baselink = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--z", "0.05", "--frame-id","base_footprint","--child-frame-id","base_link"]
    )
    tf_pub_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--z", "0.12", "--frame-id","base_link","--child-frame-id","laser"]
    )
    tf_pub_cam = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--z", "0.12", "--x","0.10","--frame-id","base_link","--child-frame-id","camera"]
    )
    
    return LaunchDescription([mycar_launch,lidar_launch,tf_pub_baselink,tf_pub_laser,tf_pub_cam,cam_launch])