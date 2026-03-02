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

        //以下均为测试代码

        //测试打印从串口读取的数据
        // serial_port_->read_print_hex();

        //测试电机速度指令写入
        // serial_port_->write_motor_speed(90,90,90,90);
        //休眠1s
        // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        //停止电机
        // serial_port_->write_motor_speed(0,0,0,0);

        //差速控制测试
        // serial_port_->write_diff_drive_control(90,90);
        // std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        // serial_port_->stop_motor();

        //pid设置
        // serial_port_->write_pid(300,0,200);//默认pid

        // serial_port_->write_pid(30,100,200);

        //读取数据测试
        // auto msg = serial_port_->read_message(my_serial::FunctionCode::VOLTAGE);//电压数据
        // if(msg){
        //     RCLCPP_INFO(this->get_logger(),"功能位: %d",msg->function_code);
        //     RCLCPP_INFO(this->get_logger(),"数据位: %d %d %d %d %d %d %d %d",
        //         msg->data[0],msg->data[1],msg->data[2],msg->data[3],
        //         msg->data[4],msg->data[5],msg->data[6],msg->data[7]);
        // }

        // auto msg2 = serial_port_->read_message(my_serial::FunctionCode::WHEEL_ENCODER);//编码器数据
        // if(msg2){
        //     RCLCPP_INFO(this->get_logger(),"功能位: %d",msg2->function_code);
        //     RCLCPP_INFO(this->get_logger(),"数据位: %d %d %d %d %d %d %d %d",
        //         msg2->data[0],msg2->data[1],msg2->data[2],msg2->data[3],
        //         msg2->data[4],msg2->data[5],msg2->data[6],msg2->data[7]);
        // }

        // auto msg3 = serial_port_->read_message(my_serial::FunctionCode::ANGULAR_VEL);//陀螺仪数据
        // if(msg3){
        //     RCLCPP_INFO(this->get_logger(),"功能位: %d",msg3->function_code);
        //     RCLCPP_INFO(this->get_logger(),"数据位: %d %d %d %d %d %d %d %d",
        //         msg3->data[0],msg3->data[1],msg3->data[2],msg3->data[3],
        //         msg3->data[4],msg3->data[5],msg3->data[6],msg3->data[7]);
        // }   

        // auto msg4 = serial_port_->read_message(my_serial::FunctionCode::ACCEL);//线加速度数据
        // if(msg4){
        //     RCLCPP_INFO(this->get_logger(),"功能位: %d",msg4->function_code);
        //     RCLCPP_INFO(this->get_logger(),"数据位: %d %d %d %d %d %d %d %d",
        //         msg4->data[0],msg4->data[1],msg4->data[2],msg4->data[3],
        //         msg4->data[4],msg4->data[5],msg4->data[6],msg4->data[7]);
        // }

        // auto msg5 = serial_port_->read_message(my_serial::FunctionCode::EULAR);//欧拉角数据
        // if(msg5){
        //     RCLCPP_INFO(this->get_logger(),"功能位: %d",msg5->function_code);
        //     RCLCPP_INFO(this->get_logger(),"数据位: %d %d %d %d %d %d %d %d",
        //         msg5->data[0],msg5->data[1],msg5->data[2],msg5->data[3],
        //         msg5->data[4],msg5->data[5],msg5->data[6],msg5->data[7]);
        // }


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