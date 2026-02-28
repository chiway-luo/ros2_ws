#ifndef MY_SERIAL_HPP
#define MY_SERIAL_HPP

#include <iostream>

/* 
    串口通信功能点分析:
        1.打开串口 -- 构造函数实现
        2.释放资源 -- 析构函数实现
        3.读操作
            3-1 一次性读取所有数据 以十六进制输出 用于测试
            3-2 读取数据,对数据进行封装,封装成对应的对象
        4.写操作
            4-1 写出电机速度指令
            4-2 写出pid参数
            4-3 写出舵机控制指令

*/
namespace my_serial{

class SerialPortComm{
public:
    //打开串口
    SerialPortComm();
    //释放资源
    ~SerialPortComm();

    //读 一次性读取所有数据 以十六进制输出 用于测试
    void read_print_hex();
    //读取数据,对数据进行解析,封装成对应的对象
    void read_message();

    //写出电机速度指令
    void write_motor_speed();
    //写出pid
    void write_pid();
    //写出舵机控制指令
    void write_servo_control();


};

SerialPortComm::SerialPortComm(){
    std::cout << "打开串口" << std::endl;
}


}




#endif // MY_SERIAL_HPP