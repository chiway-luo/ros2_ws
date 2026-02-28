#include <cstdio>
#include <iostream>
#include <boost/asio.hpp>
#include <array>
#include <string>

#include <thread>

#include <iomanip>//十六进制输出需要包含这个头文件

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
    auto fail = [&](const std::string &msg)->int{
        std::cout << msg << error.message() << std::endl;
        std::cout << "错误码: " << error.value() << std::endl;
        if(serial.is_open()){
            serial.close();
        }
        return -1;
    };
    //1-2打开串口 
    // lrwxrwxrwx   1 root root           7 Dec 18 20:31 mycar -> ttyUSB1
    // const std::string &device, boost::system::error_code &ec 参数2能够捕获异常
    serial.open("/dev/mycar",error);
    if(error){
        //如果产生异常,直接结束
        return fail("串口打开失败:");
    }

    //1-3设置串口参数(波特率,数据位,停止位,校验位)
    serial.set_option(boost::asio::serial_port_base::baud_rate(115200),error);//波特率
    if(error) return fail("设置波特率失败:");
    serial.set_option(boost::asio::serial_port_base::character_size(8),error);//数据位 一个字节 8bits
    if(error) return fail("设置数据位失败:");
    serial.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none),error);//流控(当前不使用流控)
    if(error) return fail("设置流控失败:");
    serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none),error);//校验位
    if(error) return fail("设置校验位失败:");
    serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one),error);//停止位,每一帧数据的结束符,当前使用一位停止位
    if(error) return fail("设置停止位失败:");

    //2.写 ----控制电机运动
    //2-1明确数据格式
    //FC 06 00 64 00 64 00 64 00 64 FA DF ---四个电机转速为100,100,100,100
    //FC 06 00 00 00 00 00 00 00 00 FA DF ---四个电机转速为0,0,0,0
    //数据格式说明:
    //1.帧头: 0xFC
    //2.数据长度: 0x06
    //3.电机1转速: 0x00 0x64 (100)
    //4.电机2转速: 0x00 0x64 (100)
    //5.电机3转速: 0x00 0x64 (100)
    //6.电机4转速: 0x00 0x64 (100)
    //7.校验位: 0xFA
    //8.帧尾: 0xDF
    std::array<uint8_t,12> control_data =   {0xFC, 0x06, 0x00, 0x64, 0x00, 0x64, 0x00, 0x64, 0x00, 0x64, 0xFA, 0xDF};//控制电机转速为100
    std::array<uint8_t,12> stop_data =      {0xFC, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFA, 0xDF};//控制电机转速为0

    //2-2写出数据
    //先控制电机运动
    boost::asio::write(serial,boost::asio::buffer(control_data),error);
    if(error) return fail("写控制指令失败:");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // std::this_thread::sleep_for(std::chrono::seconds(5));


    //再让电机停止
    boost::asio::write(serial,boost::asio::buffer(stop_data),error);
    if(error) return fail("写停止指令失败:");

    //3.读 --- 读取下位机发送的市局,并以十六进制的格式输出在终端
    //3-1 创建缓冲区
    std::array<uint8_t,512> recv_buffer;//接收缓冲区

    //3-2 将数据读入到缓冲区
    boost::asio::read(serial,boost::asio::buffer(recv_buffer),error);
    if(error) return fail("读取数据失败:");
    
    //3-3 写出数据
    std::cout << "接收到的数据: ";
    for(auto arg : recv_buffer){
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(arg) << " ";
    }
    std::cout << std::endl;
    /* 写出的数据

    帧头FC 功能位 电池电压高位 电池电压低位 00 00 00 00 00 00 异或校验位(第十一位) 结束位DF
        fc 05 00 00 00 00 00 3d 00 00 c4 df 
        fc 03 00 00 00 00 00 00 00 00 ff df 
        fc 04 00 01 00 00 00 62 00 00 9b df 
        fc 05 00 00 00 00 00 3d 00 00 c4 df 
        fc 03 00 00 00 00 00 00 00 00 ff df 
        fc 04 00 00 
        fc 05 00 00 00 00 00 5a 00 00 a3 df 
        fc 03 00 00 00 00 00 00 00 00 ff df 
        fc 04 00 01 00 00 00 63 00 00 9a df 
        fc 05 00 00 00 00 00 5a 00 00 a3 df 
        fc 03 00 00 00 00 00 00 00 00 ff df 
        fc 04 00 01 00 00 00 62 00 00 9b df 
        fc 05 00 00 00 00 00 5a 00 00 a3 df 
        fc 03 00 00 00 00 00 00 00 00 ff df 
        fc 04 00 01 00 00 00 63 00 00 9a df 
        fc 02 62 8f 61 e0 61 76 63 76 90 df 
        fc 05 00 00 00 00 00 5a 00 00 a3 df 
        fc 03 00 00 00 00 00 00 00 00 ff df 
        fc 04 00 01 00 00 00 61 00 00 98 df 
        fc 05 00 00 00 00 00 5a 00 00 a3 df 
        fc 03 00 00 00 00 00 00 00 00 ff df 
        fc 04 00 02 00 00 00 62 00 00 98 df 
        fc 05 00 00 00 00 00 5a 00 00 a3 df 
        fc 03 00 00 00 00 00 00 00 00 ff df 
        fc 04 00 01 00 00 00 63 00 00 9a df 
        fc 05 00 00 00 00 00 5a 00 00 a3 df 
        fc 03 00 00 00 00 00 00 00 00 ff df 
        fc 04 00 01 00 00 00 63 00 00 9a df 
        fc 02 62 ab 61 fd 61 92 63 92 a9 df 
        fc 05 00 00 00 00 00 5a 00 00 a3 df 
        fc 03 00 00 00 00 00 00 00 00 ff df 
        fc 04 00 02 00 00 00 62 00 00 98 df 
        fc 05 00 00 00 00 00 5a 00 00 a3 df 
        fc 03 00 00 00 00 00 00 00 00 ff df 
        fc 04 00 01 00 00 00 63 00 00 9a df 
        fc 05 00 00 00 00 00 5a 00 00 a3 df 
        fc 03 00 00 00 00 00 00 00 00 ff df 
        fc 04 00 00 00 00 00 62 00 00 9a df 
        fc 05 00 00 00 00 00 5a 00 00 a3 df 
        fc 03 00 00 00 00 00 00 00 00 ff df 
        fc 04 00 00 00 00 00 63 00 00 9b df 
        fc 02 62 f6 62 48 61 de 63 de 42 df 
        fc 05 00 00 00 00 00 5a 00 00 a3 df 
        fc 03 00 00 
    */

    //释放资源
    serial.close();
    return 0;
}
