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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
# 分组相关----------------------
from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction
# 事件相关----------------------
# from launch.event_handlers import OnProcessStart, OnProcessExit
# from launch.actions import ExecuteProcess, RegisterEventHandler,LogInfo
# 获取功能包下share目录路径-------
from ament_index_python.packages import get_package_share_directory
# urdf文件处理相关--------------
# from launch_ros.parameter_descriptions import ParameterValue
# from launch.substitutions import Command
"""
    注意 当前现实条件下没有多辆车的测试环境
    只是演示多辆车的launch文件编写方法
    具体参数需要在多机器人环境下测试修改

    需求: 发布两辆车的odom 相对于 map 的静态坐标系变换
            1.不同机器人的相同模块要有唯一的坐标系标识
                odom1 base_footprint1 laser1 camera1
                odom2 base_footprint2 laser2 camera2
            2.不同机器人要发布相对于map的坐标系变换
                map -> odom1
                map -> odom2
            3.需要为机器人发布订阅的话题设置命名空间
                /car1
                /car2
            
    实现: 
            1.每个机器人都需要创建一个针对编队而实现的功能包
            2.在该机器人的功能包中要重新设置坐标系名称,发布里程计相对于map的坐标变换,并为当前机器人节点设置命名空间
            3.分模块实现
            

"""
def generate_launch_description():
    #设置车辆编号
    car_id = "1" #需要同时修改params 文件夹中的部分参数
    try:#我确实没设置环境变量
        car_id = os.environ["CAR_ID"]  #从环境变量中获取车辆编号
    except KeyError:
        pass
    #读取摄像头是否使用的环境变量
    usb_cam = "1"  #默认使用摄像头
    try:
        usb_cam = os.environ["USE_CAM"]          #1/0
    except KeyError:
        pass


    # 1. 启动机器人的各个模块(不要包含功能包提供的launch文件,那都是针对单机而设置的,现在直接调用节点并设计参数)
    ld = LaunchDescription()

    #解析环境变量
    mycar_model = os.environ["MYCAR_MODEL"]  #stm32_2w stm32_4w arduino
    lidar_model = os.environ["LIDAR"]        #sl_A1 ls_N10

    
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
            parameters=[os.path.join(get_package_share_directory("mycar_bringup_multi"), "params", mycar_model + ".yaml")],#需要进入文件修改odom和base_link的编号
        )
    else:
        print("底盘类型环境变量配置错误,请检查!")
        return LaunchDescription()

    ld.add_action(base_driver)
    
    

    #集成激光雷达,直接包含launch文件
    # /home/chiway/ros2_exercise/src/mycar/laser/sllidar_ros2/launch/sllidar_launch.py
    mylaser_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sllidar_ros2'),'launch','sllidar_launch.py'
            )
        ),
#         需要传递的参数
# channel_type =  LaunchConfiguration('channel_type', default='serial')
# serial_port = LaunchConfiguration('serial_port', default='/dev/rplidar')
# serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200') #for A1/A2 is 115200
# frame_id = LaunchConfiguration('frame_id', default='laser')
# inverted = LaunchConfiguration('inverted', default='false')
# angle_compensate = LaunchConfiguration('angle_compensate', default='true')
# scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')
        launch_arguments={
            ('channel_type','serial'),
            ('serial_port','/dev/rplidar'),
            ('serial_baudrate','115200'),
            ('frame_id','laser' + car_id),
            ('inverted','false'),
            ('angle_compensate','true'),
            ('scan_mode','Sensitivity')
        }
    )
    ld.add_action(mylaser_launch)
    
    #摄像头

    params_path = os.path.join(
        get_package_share_directory('mycar_bringup_multi'),
        'params',
        'camera',
        'params.yaml'
    )#需要进入param文件修改camera编号

    camera =  Node(
        package='usb_cam', executable='usb_cam_node_exe', output='screen',
        name="cam",
        parameters=[params_path]
    )
    
    if int(usb_cam) == 1:
        ld.add_action(camera)
    else:
        print("不使用摄像头驱动")

    # 2. 发布坐标变换
    odom_to_map = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="odom_to_map1",
        arguments=[
            "--frame-id","map",
            "--child-frame-id","odom" + car_id,
            "--x","1",
            "--y","-0.5",
            "--z","0",
            "--roll","0",
            "--pitch","0",
            "--yaw","0"
        ]
    )
    base_to_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_base_footprint',
        arguments=["--frame-id","base_footprint" + car_id,"--child-frame-id","base_link" + car_id,"--z","0.06"]
    )
    laser_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_to_base_link',
        arguments=["--frame-id","base_link" + car_id,"--child-frame-id","laser" + car_id,"--z","0.06"]
    )
    camera_to_base_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_to_base_link",
        arguments=["--frame-id","base_link" + car_id,"--child-frame-id","camera" + car_id,"--x","0.165","--z","-0.035"]
    )
    ld.add_action(odom_to_map)
    ld.add_action(base_to_base_footprint)
    ld.add_action(laser_to_base_link)
    ld.add_action(camera_to_base_link)
#########################################################################
    # 3. 以分组的方式为当前机器人各个模块设置相同的命名空间

    groupOfmycar = GroupAction(
        [PushRosNamespace("mycar_lkw_4w_" + car_id),*ld.entities]
    )

    return LaunchDescription([groupOfmycar])

