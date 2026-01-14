#install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}) cmake配置
#<exec_depend>ros2launch</exec_depend> package.xml配置
#from glob import glob 用于setup.py配置多个launch文件
#('share/' + package_name, glob('launch/*_launch.py')),
#('share/' + package_name, glob('launch/*_launch.xml')),
#('share/' + package_name, glob('launch/*_launch.yaml')),

from launch import LaunchDescription
from launch_ros.actions import Node
import os
# 封装终端指令相关类--------------
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable   #FindExecutable(name="ros2")
# 参数声明与获取-----------------
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
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
    有条件的集成: 底盘驱动 雷达驱动 摄像头驱动的launch文件

    条件:
        底盘: stm32 和 arduino
        雷达: SL_A1 和 LS_N10
        摄像头: 有 和 无
    要求:
        launch文件尽量通用
    实现:
        1.以共享的环境变量存储机器人的各项参数
        2.在launch文件中解析这些环境变量,并根据解析结果动态的包含对应的驱动文件

        #配置底盘类型
        #export MYCAR_MODEL=stm32_2w
        export MYCAR_MODEL=stm32_4w
        #export MYCAR_MODEL=arduino

        #配置雷达类型
        export LIDAR=sl_A1
        # export LIDAR=ls_N10

        #配置摄像头是否使用1/0
        export USB_CAM=1
"""
def generate_launch_description():
    #解析环境变量
    mycar_model = os.environ["MYCAR_MODEL"]  #stm32_2w stm32_4w arduino
    lidar_model = os.environ["LIDAR"]        #sl_A1 ls_N10
    usb_cam = os.environ["USE_CAM"]          #1/0
    #创建临时变量存储底盘的驱动文件路径
    driver_launch_file = ""
    #根据解析的mycar_model 结果生成驱动文件路径
    if mycar_model == "arduino":
        driver_launch_file = os.path.join(
            get_package_share_directory('ros2_arduino_bridge'),
            'launch',
            'ros2_arduino_bridge.launch.py'
        )
    elif mycar_model == "stm32_4w" or mycar_model == "stm32_2w":
        driver_launch_file = os.path.join(
            get_package_share_directory('ros2_stm32_bridge'),
            'launch',
            'driver.launch.py'
        )
    else:
        print("底盘类型环境变量配置错误,请检查!")
        return LaunchDescription()
    
    ld = LaunchDescription()
    #底盘节点
    mycar_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            # os.path.join(
            #     get_package_share_directory('ros2_stm32_bridge'),'launch','driver.launch.py'
            # )
            driver_launch_file
        )
    )
    ld.add_action(mycar_launch)
    #摄像头
    if int(usb_cam) == 1:
        mycamera_launch = IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('mycar_cam'),'launch','usb_cam.launch.py'
                )
            )
        )
        ld.add_action(mycamera_launch)
    else:
        print("不使用摄像头驱动")
    #雷达
    mylaser_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sllidar_ros2'),'launch','sllidar_launch.py'
            )
        )
    )
    ld.add_action(mylaser_launch)

    #发布坐标变换实现
    #当设置传感器相对位姿关系时,需要参考base_link坐标系 而不是 base_footprint
    base_to_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_base_footprint',
        arguments=["--frame-id","base_footprint","--child-frame-id","base_link","--z","0.06"]
    )
    laser_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_to_base_link',
        arguments=["--frame-id","base_link","--child-frame-id","laser","--z","0.06"]
    )
    camera_to_base_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_to_base_link",
        arguments=["--frame-id","base_link","--child-frame-id","mycamera_front","--x","0.165","--z","-0.035"]
    )
    ld.add_action(base_to_base_footprint)
    ld.add_action(laser_to_base_link)
    if int(usb_cam) == 1:
        ld.add_action(camera_to_base_link)


    #添加mycar_description的机器人描述文件 添加内容
    mycar_description_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mycar_description'),'launch','mycar_desc.launch.py'
            )
        )
    )
    ld.add_action(mycar_description_launch)

    #添加雷达稳定频率节点
    # lidar_freq_node = Node(
    #     package="laser_rate_control",
    #     executable="sllidar_a1_control"
    # )
    # ld.add_action(lidar_freq_node)

    # mycar_map = IncludeLaunchDescription(
    #     launch_description_source=PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('mycar_slam_gmapping'),'launch','gmapping.launch.py'
    #         )
    #     )
    # )
    # mycar_map = IncludeLaunchDescription(
    #     launch_description_source=PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('mycar_slam_slam_toolbox'),'launch','online_async_launch.py'
    #         )
    #     )
    # )
    # ld.add_action(mycar_map)

    # mycar_nav = IncludeLaunchDescription(
    #     launch_description_source=PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('mycar_nav2'),'launch','nav2.launch.py'
    #         )
    #     )
    # )
    # ld.add_action(mycar_nav)

    return ld








# urdf实例代码:
"""
# 1.启动robot_satte_publisher节点,该节点要以参数的方式加载urdf文件内筒;
p_value = ParameterValue(value=Command(["xacro ",get_package_share_directory('cpp06_urdf') + '/urdf/urdf/demo01_helloworld.urdf']),value_type=str)
robot_state_pub = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': p_value}]
)
"""