#install(DIRECTORY config params launch DESTINATION share/${PROJECT_NAME}) #cmake配置
#<exec_depend>ros2launch</exec_depend> <!--package.xml配置-->
#from glob import glob #用于setup.py配置多个launch文件
#('share/' + package_name + '/launch', glob('launch/*launch.py')),
#('share/' + package_name + '/launch', glob('launch/*launch.xml')),
#('share/' + package_name + '/launch', glob('launch/*launch.yaml')),
#(os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

from launch import LaunchDescription
from launch_ros.actions import Node
import os
# 封装终端指令相关类--------------
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable   #FindExecutable(name="ros2")
# 参数声明与获取-----------------
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# from launch.conditions import IfCondition #判断是否执行
# from launch.conditions import UnlessCondition #取反
# from launch.substitutions import PythonExpression #运行时计算表达式
# 文件包含相关-------------------
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# 分组相关----------------------
# from launch_ros.actions import PushRosNamespace
# from launch.actions import GroupAction
# 事件相关----------------------
# from launch.event_handlers import OnProcessStart, OnProcessExit
# from launch.actions import ExecuteProcess, RegisterEventHandler,LogInfo
# 获取功能包下share目录路径-------
# from ament_index_python.packages import get_package_share_directory
# urdf文件处理相关--------------
# from launch_ros.parameter_descriptions import ParameterValue
# from launch.substitutions import Command
"""
注意:该launch文件需要使用多线激光雷达,但是手上没有,所以没有测试

    实现: 将多线激光雷达数据(点云数据)转换为二维激光雷达数据(激光扫描数据)
    说明: 使用pointcloud_to_laserscan包中的pointcloud_to_laserscan节点
    输入: PointCloud2类型的点云数据
    输出: LaserScan类型的激光扫描数据
"""
def generate_launch_description():
    ld = LaunchDescription()

    #启动多线激光雷达节点 没有
    # point_cloud_laser_node = Node(
    #     package='rsliar_sdk',
    #     executable='start.py',
    #     name='rsliar_sdk',
    #     output='screen',
    # )
    # ld.add_action(point_cloud_laser_node)

    #启动点云数据转换为激光扫描数据节点
    pointcloud2_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_node',
        output='screen',
        parameters=[{
            'target_frame': 'base_link',          #目标坐标系
            # 'transform_tolerance': 0.01,          #坐标变换容忍时间
            'min_height': -0.1,                    #激光扫描最小高度
            'max_height': 0.1,                     #激光扫描最大高度
            # 'angle_min': -3.14,                    #激光扫描最小角度
            # 'angle_max': 3.14,                     #激光扫描最大角度
            # 'angle_increment': 0.00436332309619,   #激光扫描角度增量
            # 'scan_time': 0.1,                      #激光扫描时间
            # 'range_min': 0.0,                      #激光扫描最小范围
            # 'range_max': 30.0,                     #激光扫描最大范围
            # 'use_inf': True,                       #使用无穷大表示超出范围的值
            # 'inf_epsilon': 1.0,                    #无穷大偏移量
            # 'input_pointcloud_topic': '/points_raw',  #输入点云数据话题
            # 'output_laserscan_topic': '/scan'         #输出激光扫描数据话题
        }],
        remappings=[
            ('/cloud_in', '/points_raw'),  #输入点云数据话题重映射
            ('/scan', '/scan')              #输出激光扫描数据话题重映
        ]
    )
    ld.add_action(pointcloud2_to_laserscan_node)

    return ld


