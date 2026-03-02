#ifndef MY_SERIAL_HPP
#define MY_SERIAL_HPP

#include <iostream>

#include <boost/asio.hpp>
#include <string>

#include <iomanip>//十六进制输出
/* 
    串口通信功能点分析:
        1.打开串口 -- 构造函数实现
        2.释放资源 -- 析构函数实现
        3.读操作
            3-1 一次性读取一定量的数据 以十六进制输出 用于测试
            3-2 读取数据,对数据进行封装,封装成对应的对象
        4.写操作
            4-1 写出电机速度指令
            4-2 写出pid参数
            4-3 写出舵机控制指令

*/
namespace my_serial{

//封装当前解析状态
enum ParseStatus{//解析状态枚举类
    WAITING_FOR_HEADER, //等待帧头
    READING_FUNCTION,   //读取功能位
    READING_DATA,       //读取数据位
    READING_CHECKSUM,   //读取校验位
    READING_END         //读取结束位
};
enum FunctionCode{//功能位枚举类
    VOLTAGE = 0x01, //电压数据
    WHEEL_ENCODER = 0x02, //编码器数据(轮速)
    ANGULAR_VEL = 0x03, //陀螺仪数据(角速度)
    ACCEL = 0x04, //线加速度数据
    EULAR = 0x05, //欧拉角数据
    // MOTOR_SPEED = 0x06, //电机速度控制
    // PID_CONTROL = 0x07, //PID参数控制
    // SERVO_CONTROL = 0x08 //舵机控制
};
struct Message{//消息类型
    FunctionCode function_code;//功能位
    std::vector<uint8_t> data;//数据位
};

    



class SerialPortComm{
public:
    //打开串口
    SerialPortComm(const std::string& port,int baud_rate,int character_size);//串口号,波特率,数据位
    //释放资源
    ~SerialPortComm();

    //读 一次性读取一定量的数据 以十六进制输出 用于测试
    void read_print_hex();

    //读取数据,对数据进行解析,封装成对应的对象
    inline std::shared_ptr<Message> read_message(FunctionCode need_function_code);

    //写出电机速度指令
    void write_motor_speed(short motor1_speed,short motor2_speed,short motor3_speed,short motor4_speed);
    //电机控制重载函数
    void write_motor_speed(short motor_a,short motor_b);

    //写出pid
    void write_pid(short kp, short ki, short kd);
    //写出舵机控制指令
    void write_servo_control(int8_t servo1_angle, int8_t servo2_angle);

    //异常处理函数
    int handle_error(const std::string &msg,const boost::system::error_code &error);

    //校验操作函数 被校验数组
    void calculate_checksum(std::array<uint8_t, 12> &data);

    //差速小车电机控制
    void write_diff_drive_control(short motor_a,short motor_b){
        write_motor_speed(motor_a,motor_b,motor_a,motor_b);
    }

    //停止电机运动函数
    void stop_motor(){
        write_motor_speed(0,0,0,0);
    }

    //获取pid频率的函数
    int get_pid_rate() const{
        return pid_rate_;
    }

private:
    boost::asio::io_context io;//io上下文对象
    boost::asio::serial_port serial;//串口通信对象
    boost::system::error_code error;//异常处理对象

    //存储当前解析状态 默认值从查找帧头开始
    ParseStatus current_status_;
    //创建缓存数组,用于存储分析读取的数据
    std::array<uint8_t, 512> buffer_;
    //创建一个存储message数据的数组
    std::array<uint8_t, 8> data_;
    //用于校验
    uint8_t check_num_ = 0xFC;
    //设置一个用于存储消息功能类型的变量
    FunctionCode current_function_code_;

    int pid_rate_;//pid频率,单位hz
    

};

//串口号 /dev/mycar
SerialPortComm::SerialPortComm(const std::string& port,int baud_rate,int character_size) : 
    io(), serial(io), current_status_(WAITING_FOR_HEADER), check_num_(0xFC), pid_rate_(25){//初始化串口对象

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

//从串口读取数据到缓冲区,并以十六进制格式打印
void SerialPortComm::read_print_hex(){
    std::array<uint8_t, 512> buffer;//缓冲区
    boost::asio::read(serial, boost::asio::buffer(buffer), error);
    if (error){
        handle_error("读取数据失败: ", error);
        return;
    }
    //遍历打印
    for(auto arg : buffer){
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(arg) << ' ';
        if(arg == 0xDF){
            std::cout << std::endl;
        }
    }
}

//读取数据,对数据进行解析,封装成对应的对象
inline std::shared_ptr<Message> SerialPortComm::read_message(FunctionCode need_function_code){
    //读数据需求
    //将有效数据帧封装并返回

    //先获取帧头
    //再获取功能位
    //读取校验位
    //读取结束位

    //根据功能位解析数据 查询帧头,功能位
    //解析的状态封装 --- 查询帧头 功能位 数据位
    //功能位封装 --- 电压 编码器 陀螺仪数据 ...
    //消息对象 ---- 功能位类型 以及 数据位
    while (serial.is_open())
    {
        switch (current_status_)
        {
        case WAITING_FOR_HEADER://查找帧头
            //从串口读取一个字节,查找是否是帧头
            boost::asio::read(serial, boost::asio::buffer(buffer_, 1), error);
            if(error){
                handle_error("读取数据失败: ", error);
                return nullptr;
            }
            //判断是否是帧头
            if(buffer_[0] == 0xFC){
                current_status_ = READING_FUNCTION;//更新状态为读取功能位
                check_num_ = buffer_[0];//将帧头存储到校验变量中,用于后续校验
            }
            else{
                break;
            }
            // fall through
        case READING_FUNCTION://读取功能位
            //从串口读取一个字节
            boost::asio::read(serial, boost::asio::buffer(buffer_, 1), error);
            if(error){
                handle_error("读取数据失败: ", error);
                return nullptr;
            }
            //判断功能位是否合法 且符合查找要求
            if (buffer_[0] >= 0x01 && buffer_[0] <= 0x05 && buffer_[0] == static_cast<uint8_t>(need_function_code)){
                current_status_ = READING_DATA;//更新状态为读取数据位
                check_num_ ^= buffer_[0];//将功能位与校验变量进行异或运算,更新校验变量的值
                current_function_code_ = static_cast<FunctionCode>(buffer_[0]);//将功能位存储到当前功能位变量中,用于后续解析数据
            }else{
                current_status_ = WAITING_FOR_HEADER;//如果功能位不合法,重新查找帧头
                break;
            }
            // fall through
        case READING_DATA://读取数据位
            boost::asio::read(serial, boost::asio::buffer(&data_[0], 8), error);
            if (error)
            {
                handle_error("读取数据失败: ", error);
                return nullptr;
            }
            //异或校验
            for (size_t i = 0; i < 8; ++i){
                check_num_ ^= data_[i];
            }
            //设置状态
            current_status_ = READING_CHECKSUM;//更新状态为读取校验位
            // fall through
        case READING_CHECKSUM://读取校验位
            boost::asio::read(serial, boost::asio::buffer(&buffer_[0], 1), error);
            if (error)        {
                handle_error("读取数据失败: ", error);
                return nullptr;
            }
            //判断校验位是否和我们的一致
            if (buffer_[0] == check_num_){
                //校验成功
                current_status_ = READING_END;//更新状态为读取结束位
            }else{
                //校验失败
                current_status_ = WAITING_FOR_HEADER;//更新状态为等待帧头
                break;
            }
            // fall through
        case READING_END://读取结束位
            current_status_ = WAITING_FOR_HEADER;//无论成功与否都要重新查找帧头
            boost::asio::read(serial, boost::asio::buffer(&buffer_[0],1), error);
            if (error){
                handle_error("读取数据失败: ", error);
                return nullptr;
            }
            //判断结束位是否正确
            if (buffer_[0] == 0xDF){
                //组织并返回对象
                std::shared_ptr<Message> msg = std::make_shared<Message>();
                msg->function_code = current_function_code_;//设置功能位
                msg->data = std::vector<uint8_t>(data_.begin(), data_.end());//设置数据位
                return msg;
            }
            break;
        default:
            current_status_ = WAITING_FOR_HEADER;
            break;
        }
        
    }
    

    
    return nullptr;

}


//写出电机速度指令 左前轮 右前轮 左后轮 右后轮
void SerialPortComm::write_motor_speed(short motor_a,short motor_b,short motor_c,short motor_d){
    /* 
        返回值: void 单向流动的数据
        参数: 四个电机的速度
        
        需要将每个电机的速度转换成两个数据位并发送
    */
    //创建字节数组
    std::array<uint8_t, 12> data = {0xFC, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFA, 0xDF};
    //设置字节数组的元素
    data[2] = (motor_a >> 8) & 0xff;//电机a的高八位,右移八位,再与0xFF按位与运算,得到高八位的值
    data[3] = motor_a & 0xff;//电机a的低八位,与0xFF按位与运算,得到低八位的值

    data[4] = (motor_b >> 8) & 0xff;//电机b的高八位
    data[5] = motor_b & 0xff;//电机b的低八位

    data[6] = (motor_c >> 8) & 0xff;//电机c的高八位
    data[7] = motor_c & 0xff;//电机c的低八位

    data[8] = (motor_d >> 8) & 0xff;//电机d的高八位
    data[9] = motor_d & 0xff;//电机d的低八位

    // for(size_t i = sign_num; i < sign_num + 4; ++i){
    //     data[i] = (i % 2 == 0) ? ((motor_a >> 8) & 0xff) : (motor_a & 0xff);
    // }

    //校验操作
    calculate_checksum(data);

    //写出数据
    boost::asio::write(serial, boost::asio::buffer(data,12), error);
    if(error){
        handle_error("写出数据失败: ", error);
        return;
    }

   
}


//写出pid
void SerialPortComm::write_pid(short kp, short ki, short kd){
    //数据格式 FC 07 p高 p低 i i d d 0 0 FA DF
    //设置电机控制的pid参数

    //设置字节数组
    std::array<uint8_t, 12> data = {0xFC, 0x07, 0x01, 0x2c, 0x00, 0x00, 0x00, 0xC8, 0x00, 0x00, 0x1E, 0xDF};
    data[2] = (kp >> 8) & 0xff;//kp的高八位
    data[3] = kp & 0xff;//kp的低八位

    data[4] = (ki >> 8) & 0xff;//ki的高八位
    data[5] = ki & 0xff;//ki的低八位

    data[6] = (kd >> 8) & 0xff;//kd的高八位
    data[7] = kd & 0xff;//kd的低八位

    //校验
    calculate_checksum(data);

    //写出数据
    boost::asio::write(serial, boost::asio::buffer(data,12), error);
    if(error){
        handle_error("写出pid失败: ", error);
        return;
    }
}

//写出舵机控制指令
void SerialPortComm::write_servo_control(int8_t servo1_angle, int8_t servo2_angle){
    /* 
        参数列表: 帧头 功能位08 舵机1角度 舵机2角度 00 00 00 00 00 00 校验位 帧尾

    */
    std::array<uint8_t, 12> data = {0xFC, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF4, 0xDF};
    data[2] = servo1_angle;
    data[3] = servo2_angle;
    //校验
    calculate_checksum(data);

    //写出数据
    boost::asio::write(serial, boost::asio::buffer(data,12), error);
    if(error){
        handle_error("写出舵机控制指令失败: ", error);
        return;
    }
}


//校验操作函数 被校验数组
void SerialPortComm::calculate_checksum(std::array<uint8_t, 12> &data){
    uint8_t check_sum = 0;
    for(size_t i = 0; i < data.size() - 2; ++i){
        check_sum ^= data[i];
    }
    data[10] = check_sum;//校验位
}






}//namespace my_serial



#endif // MY_SERIAL_HPP