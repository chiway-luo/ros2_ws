#install(DIRECTORY config params launch DESTINATION share/${PROJECT_NAME}) #cmake配置
#<exec_depend>ros2launch</exec_depend> <!--package.xml配置-->
#from glob import glob #用于setup.py配置多个launch文件
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
# from launch.conditions import IfCondition #判断是否执行
# from launch.conditions import UnlessCondition #取反
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
from ament_index_python.packages import get_package_share_directory
# urdf文件处理相关--------------
# from launch_ros.parameter_descriptions import ParameterValue
# from launch.substitutions import Command
"""
    当前launch文件使用的param设置了命名空间,但是没有对节点使用命名空间,所以雷达会停止转动
"""
def generate_launch_description():

    ld = LaunchDescription()

    #解析环境变量
    mycar_model = os.environ["MYCAR_MODEL"]  #stm32_2w stm32_4w arduino
    lidar_model = os.environ["LIDAR"]        #sl_A1 ls_N10
    usb_cam = os.environ["USE_CAM"]          #1/0

    
    #底盘节点
    #根据解析的mycar_model 启动对应节点  /home/chiway/ros2_exercise/src/mycar/integration/mycar_bringup_multi
    base_driver = None
    if mycar_model == "arduino":
        base_driver = Node(
            package="ros2_arduino_bridge", 
            executable="arduino_node", 
            name="ros2_arduino_node",
            parameters=[os.path.join(get_package_share_directory("mycar_bringup_multi"),"params","arduino.yaml")],
        )
    elif mycar_model == "stm32_4w" or mycar_model == "stm32_2w":
        base_driver = Node(
            package="ros2_stm32_bridge",
            executable="base_controller",
            parameters=[os.path.join(get_package_share_directory("mycar_bringup_multi"), "params", mycar_model + ".yaml")],
        )
    else:
        print("底盘类型环境变量配置错误,请检查!")
        return LaunchDescription()

    ld.add_action(base_driver)

    return ld
