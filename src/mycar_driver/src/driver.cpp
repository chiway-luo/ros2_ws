#include "rclcpp/rclcpp.hpp"

#include "mycar_driver/my_serial.hpp"
/*
    需求:使用串口通信类结合ros2实现小车的底盘驱动
    流程:
        1.创建底盘里程计发布节点
        2.创建陀螺仪发布节点

    功能点:
        1.订阅速度指令并控制机器人运动
        2.发布电池电压相关数据
        3.发布编码器检测值
        4.计算并发布里程计数据
        5.结合角速度/线加速度/欧拉角数据,生成并发布imu消息
        6.实现pid参数的动态调整
    
*/

using namespace std::chrono_literals; //使用时间命名空间
using namespace std::placeholders; //占位符命名空间

class MyCarDriver :public rclcpp::Node{
public:
    MyCarDriver(std::string str1):Node(str1){
        RCLCPP_INFO(this->get_logger(),"namesapce:  node: %s 节点创建成功",str1.c_str());

        this->declare_parameter<std::string>("port", "/dev/mycar");
        this->declare_parameter<int>("baud_rate", 115200);
        this->declare_parameter<int>("data_bits", 8);

        const auto port = this->get_parameter("port").as_string();
        const auto baud_rate = this->get_parameter("baud_rate").as_int();
        const auto data_bits = this->get_parameter("data_bits").as_int();

        //实例化串口通信对象
        serial_port_ = std::make_shared<my_serial::SerialPortComm>(port, baud_rate, data_bits);

        RCLCPP_INFO(
            this->get_logger(),
            "串口参数: port=%s, baud_rate=%d, data_bits=%d",
            port.c_str(),
            baud_rate,
            data_bits);

        

    }

private:
    std::shared_ptr<my_serial::SerialPortComm> serial_port_;//串口通信对象指针
};

int main(int argc, char * argv[])
{
    //初始化ros2客户端
    rclcpp::init(argc,argv);

    //调用spin函数,使用自定义类对象指针
    rclcpp::spin(std::make_shared<MyCarDriver>("mycar_driver_cpp"));//node_name, (namespace可选)

    //释放资源
    rclcpp::shutdown();
    return 0;
}