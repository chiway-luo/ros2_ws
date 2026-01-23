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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
# 分组相关----------------------
# from launch_ros.actions import PushRosNamespace
# from launch.actions import GroupAction
# 事件相关----------------------
# from launch.event_handlers import OnProcessStart, OnProcessExit
# from launch.actions import ExecuteProcess, RegisterEventHandler,LogInfo
# 获取功能包下share目录路径-------
from ament_index_python.packages import get_package_share_directory
# urdf文件处理相关--------------
# from launch_ros.parameter_descriptions import ParameterValue
# from launch.substitutions import Command
"""
由于当前手上没有多余的激光雷达设备,假设现在有两个激光雷达, 
映射名称分别为laser_up laser_down  话题分别为 /scan_up 和 /scan_down 话题,

将它们的数据合并后发布到 /scan_merged 话题上.


    需求: 将多个激光雷达数据合并为一个话题发布
    依赖: ira_laser_tools

"""
def generate_launch_description():
    ld = LaunchDescription()

    #上方激光雷达节点
    """
        serial_port = LaunchConfiguration('serial_port', default='/dev/rplidar')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200') #for A1/A2 is 115200
    frame_id = LaunchConfiguration('frame_id', default='laser')
    """
    # laser_up_node = IncludeLaunchDescription(
    #     launch_description_source=PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('sllidar_ros2'),
    #             'launch',
    #             'sllidar_launch.py'
    #         )
    #     ),
    #     launch_arguments={
    #         'serial_port': '/dev/laser_up',
    #         'frame_id': 'laser_up',
    #         'node_name': 'sllidar_up',
    #         'angle_compensate': 'true'
    #     }.items(),
    #     remapments={
    #         '/scan': '/scan_up'
    #     }
    # )
    laser_up_node = Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node_up',
            parameters=[
                {
                    'channel_type': 'serial',
                    'serial_port': '/dev/laser_up', 
                    'serial_baudrate': 115200, 
                    'frame_id': 'laser_up',
                    'inverted': False, 
                    'angle_compensate': True
                }
            ],
            remappings=[
                ('/scan', '/scan_up')
            ],
            output='screen'
    )
    ld.add_action(laser_up_node)

    #下方激光雷达节点
    # laser_down_node = IncludeLaunchDescription(
    #     launch_description_source=PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('sllidar_ros2'),
    #             'launch',
    #             'sllidar_launch.py'
    #         )
    #     ),
    #     launch_arguments={
    #         'serial_port': '/dev/laser_down',
    #         'frame_id': 'laser_down',
    #         'node_name': 'sllidar_down',
    #         'angle_compensate': 'true'
    #     }.items(),
    #     remapments={
    #         '/scan': '/scan_down'
    #     }
    # )
    laser_down_node = Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node_down',
            parameters=[
                {
                    'channel_type': 'serial',
                    'serial_port': '/dev/laser_down', 
                    'serial_baudrate': 115200, 
                    'frame_id': 'laser_down',
                    'inverted': False, 
                    'angle_compensate': True
                }
            ],
            remappings=[
                ('/scan', '/scan_down')
            ],
            output='screen'
    )
    ld.add_action(laser_down_node)

    #发布激光雷达坐标系相对于base_link的静态变换
    laser_down_to_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_down_to_base_tf',
        arguments=[
            '--frame-id','base_link', 
            '--child-frame-id', 'laser_down', 
            '--x', '0.0', 
            '--y', '0.0', 
            '--z','-0.05', 
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0'
        ]
    )
    ld.add_action(laser_down_to_base_tf)

    laser_up_to_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_up_to_base_tf',
        arguments=[
            '--frame-id','base_link', 
            '--child-frame-id', 'laser_up', 
            '--x', '0.0', 
            '--y', '0.0', 
            '--z','0.05', 
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0'
        ]
    )
    ld.add_action(laser_up_to_base_tf)

    #启动融合节点
    """
        <param name="destination_frame" value="base_link"/>
		<param name="cloud_destination_topic" value="/merged_cloud"/>
		<param name="scan_destination_topic" value="/scan_multi"/>
		<param name="laserscan_topics" value ="/scansx /scandx" />
		<!-- LIST OF THE LASER SCAN TOPICS TO SUBSCRIBE -->
		<param name="angle_min" value="-3.14"/>
		<param name="angle_max" value="3.14"/>
		<param name="angle_increment" value="0.00437"/>
		<param name="scan_time" value="0.0"/>
		<param name="range_min" value="0.1"/>
		<param name="range_max" value="2.0"/>
    """
    # laser_merger_node = IncludeLaunchDescription(
    #     launch_description_source=PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('ira_laser_tools'),
    #             'launch',
    #             'laserscan_multi_merger.launch'
    #         )
    #     ),
    #     launch_arguments={
    #         'destination_frame': 'base_link',
    #         'scan_destination_topic': '/scan_merged',
    #         'laserscan_topics': '/scan_up /scan_down'
    #     }
    # )
    laser_merger_node = Node(
        package='ira_laser_tools',
        executable='laserscan_multi_merger',
        name='ira_merger_node',
        parameters=[
            os.path.join(
                get_package_share_directory('mycar_laser_merger'),
                'params',
                'ira_merger.yaml'
            )
        ],
        output='screen'
    )
    ld.add_action(laser_merger_node)

    #新增 对融合后的节点进行过滤 box filtering
    laser_box_filter_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[
            os.path.join(
                get_package_share_directory("mycar_laser_merger"),
                "params", "box_filter.yaml",
            )],
        remappings=[("/scan","/scan_multi")] #发布/scan_filtered
    )
    ld.add_action(laser_box_filter_node)


    return ld
