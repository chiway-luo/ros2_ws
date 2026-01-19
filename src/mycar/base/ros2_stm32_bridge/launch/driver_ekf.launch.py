from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
from launch_ros.actions import Node

"""
    需求:实现车轮编码器里程计与imu的融合,以获取更准确的里程计消息
"""
def generate_launch_description():
    MYCAR_MODEL = os.environ['MYCAR_MODEL']
    ld = LaunchDescription()

    #启动ekf节点 实现里程计与imu的融合
    ekf_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("ros2_stm32_bridge"), 'params', 'ekf.yaml')],
        )
    ld.add_action(ekf_node)

    #发布imu_link到base_footprint的静态变换
    imu2base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--frame-id', 'base_footprint', '--child-frame-id', 'imu_link', '--x', '-0.15']
    )
    ld.add_action(imu2base)

    #启动机器人的底盘驱动
    ros2_stm32 = Node(
        package="ros2_stm32_bridge",
        executable="base_controller",
        parameters=[
            os.path.join(get_package_share_directory("ros2_stm32_bridge"), "params", MYCAR_MODEL + "_ekf.yaml"),],
    )
    ld.add_action(ros2_stm32)

    return ld