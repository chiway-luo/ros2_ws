#ifndef MY_SERIAL_HPP
#define MY_SERIAL_HPP

#include <iostream>

#include <boost/asio.hpp>
#include <string>

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
    SerialPortComm(const std::string& port,int baud_rate,int character_size);//串口号,波特率,数据位
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

    //异常处理函数
    int handle_error(const std::string &msg,const boost::system::error_code &error);

private:
    boost::asio::io_context io;//io上下文对象
    boost::asio::serial_port serial;//串口通信对象
    boost::system::error_code error;//异常处理对象
    

};

//串口号 /dev/mycar
SerialPortComm::SerialPortComm(const std::string& port,int baud_rate,int character_size) : io(), serial(io){//初始化串口对象
    std::cout << "打开串口" << std::endl;
    //打开串口
    serial.open(port,error);
    //抛出运行时异常
    if(error)   throw std::runtime_error("无法打开串口: " + port + " " + error.message());
        
        

    //设置通信参数
    serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate),error);//波特率
    if(error) throw std::runtime_error("设置波特率失败: " + error.message());
    serial.set_option(boost::asio::serial_port_base::character_size(character_size),error);//数据位 一个字节 8bits
    if(error) throw std::runtime_error("设置数据位失败: " + error.message());
    serial.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none),error);//流控(当前不使用流控)
    if(error) throw std::runtime_error("设置流控失败: " + error.message());
    serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none),error);//校验位
    if(error) throw std::runtime_error("设置校验位失败: " + error.message());
    serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one),error);//停止位,每一帧数据的结束符,当前使用一位停止位
    if(error) throw std::runtime_error("设置停止位失败: " + error.message());

    std::cout << port << " 串口打开且设置成功" << std::endl;
}

SerialPortComm::~SerialPortComm(){
    std::cout << "释放资源" << std::endl;
    if(serial.is_open()){
        serial.close();
    }
}

//异常处理函数
int SerialPortComm::handle_error(const std::string &msg,const boost::system::error_code &error){
    std::cout << msg << ' ' << error.message() << std::endl;
    std::cout << "错误码: " << error.value() << std::endl;
    if(serial.is_open()){
        serial.close();
    }
    return -1;
}

}




#endif // MY_SERIAL_HPP