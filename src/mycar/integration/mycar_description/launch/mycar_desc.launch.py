from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command,LaunchConfiguration
from launch.actions import DeclareLaunchArgument

#示例： ros2 launch cpp06_urdf display.launch.py model:=`ros2 pkg prefix --share cpp06_urdf`/urdf/urdf/demo01_helloworld.urdf
def generate_launch_description():

    MYCAR_MODEL = os.environ['MYCAR_MODEL']

    mycar_description = get_package_share_directory("mycar_description")
    default_model_path = os.path.join(mycar_description,"urdf",MYCAR_MODEL + ".urdf")
    model = DeclareLaunchArgument(name="model", default_value=default_model_path)

    # 加载机器人模型
    # 1.启动 robot_state_publisher 节点并以参数方式加载 urdf 文件
    # robot_description = ParameterValue(Command(["xacro ",LaunchConfiguration("model")]))
    robot_description = ParameterValue(
        Command(["cat ",LaunchConfiguration("model")]),
        value_type=str
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )
    # 2.启动 joint_state_publisher 节点发布非固定关节状态
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher"
    )
    
    return LaunchDescription([
        model,
        robot_state_publisher,
        joint_state_publisher,
    ])