#include <cstdio>
#include <iostream>
#include <boost/asio.hpp>

/* 
  需求:使用boost::asio 实现上位机与下位机通信
      1. 读操作 --- 读取下位机发送的数据
      2. 写操作 --- 向下位机发送数据,控制底盘电机运动

  流程:
      1.创建串口通信相关对象
      2.实现写操作
      3.实现读操作

      end: 释放资源
*/
int main(int argc, char ** argv)
{
    (void) argc;
    (void) argv;

    //1.创建串口通信相关对象(打开串口,并且设置参数,以能够与串口通信)
    //1-1创建通信对象
    boost::asio::io_context io;//boost库中的所有读写操作的核心对象,所有的异步操作都需要依赖于这个对象,管理io服务的生命周期,提供了一个事件循环机制,可以让我们在其中执行异步操作,并且在事件发生时得到通知
    boost::asio::serial_port serial(io);//专用于串口通信的对象,它是一个模板类,需要指定io_context对象作为参数,以便于进行异步操作
    boost::system::error_code error;//错误码对象,用于捕获和处理在串口通信过程中可能发生的错误,当我们执行串口操作时,如果发生错误,相关的错误信息将被存储在这个对象中,我们可以通过检查这个对象来确定操作是否成功以及获取错误的详细信息
    //1-2打开串口 
    // lrwxrwxrwx   1 root root           7 Dec 18 20:31 mycar -> ttyUSB1
    // const std::string &device, boost::system::error_code &ec 参数2能够捕获异常
    serial.open("/dev/mycar",error);
    if(error){
        //如果产生异常,直接结束
        std::cout << "串口打开失败:" << error.message() << std::endl;
        std::cout << "错误码: " << error.value() << std::endl;
        return -1;
    }

    //1-3设置串口参数(波特率,数据位,停止位,校验位)
    serial.set_option(boost::asio::serial_port_base::baud_rate(115200));//波特率
    serial.set_option(boost::asio::serial_port_base::character_size(8));//数据位 一个字节 8bits
    serial.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));//流控(当前不使用流控)
    serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));//校验位
    serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));//停止位,每一帧数据的结束符,当前使用一位停止位




    //释放资源
    serial.close();
    return 0;
}
