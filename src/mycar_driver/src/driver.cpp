#include "rclcpp/rclcpp.hpp"

#include "mycar_driver/my_serial.hpp"
/*
    需求:实现小车的底盘驱动

    阶段一: 搭建代码框架
        1.创建头文件,实现串口通信的相关功能
        2.当前源文件,调用头文件,关注的是ros2相关业务逻辑
        这样设计的优点,提高内聚性,降低耦合性,方便后期的维护和拓展
    阶段二: 编写并测试头文件功能
        1.打开以及关闭串口
        2.读实现
        3.写实现
   
*/

using namespace std::chrono_literals; //使用时间命名空间
using namespace std::placeholders; //占位符命名空间

class MyNode :public rclcpp::Node{
public:
    MyNode(std::string str1):Node(str1){
        RCLCPP_INFO(this->get_logger(),"namesapce:  node: %s 节点创建成功",str1.c_str());

        //实例化串口通信对象
        serial_port_ = std::make_shared<my_serial::SerialPortComm>("/dev/mycar",115200,8);

    }

private:
    std::shared_ptr<my_serial::SerialPortComm> serial_port_;//串口通信对象指针
};

int main(int argc, char * argv[])
{
    //初始化ros2客户端
    rclcpp::init(argc,argv);

    //调用spin函数,使用自定义类对象指针
    rclcpp::spin(std::make_shared<MyNode>("MyNode_node_cpp"));//node_name, (namespace可选)

    //释放资源
    rclcpp::shutdown();
    return 0;
}