#include "rclcpp/rclcpp.hpp"

#include "mycar_driver/my_serial.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include <atomic>
#include <condition_variable>
#include <deque>
#include <limits>
#include <mutex>
#include <thread>
#include "std_msgs/msg/u_int16.hpp"//电池电压
#include "std_msgs/msg/u_int16_multi_array.hpp"//编码器数据
#include "nav_msgs/msg/odometry.hpp"//里程计数据

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

    读取电池电压数据,并以话题的方式在ros2中发布
        使用工具类读取下位机发送的电压数据 -- 使用多线程读取数据(实现异步操作)
        获取所需要的数据帧对象
        将帧对象中的数据封装为ros2接口并发布
    
*/

using namespace std::chrono_literals; //使用时间命名空间
using namespace std::placeholders; //占位符命名空间

class MyCarDriver :public rclcpp::Node{
public:
    MyCarDriver(std::string str1):Node(str1), last_cmd_vel_time_(this->now()), flag_(true){
        RCLCPP_INFO(this->get_logger(),"namesapce:  node: %s 节点创建成功",str1.c_str());

        //声明参数
        //将部分参数设置为只读模式
        rcl_interfaces::msg::ParameterDescriptor onlyread_descriptor;
        onlyread_descriptor.read_only = true;
        this->declare_parameter<std::string>("port", "/dev/ttyUSB0", onlyread_descriptor);
        this->declare_parameter<int>("baud_rate", 115200, onlyread_descriptor);
        this->declare_parameter<int>("data_bits", 8, onlyread_descriptor);

        this->declare_parameter<std::string>("odom_frame","odom");
        this->declare_parameter<std::string>("odom_topic","/odom");
        this->declare_parameter<std::string>("imu_frame","imu");
        this->declare_parameter<std::string>("imu_topic","/imu");
        this->declare_parameter<std::string>("cmd_vel_topic","/cmd_vel");
        this->declare_parameter<double>("wheel_diameter",0.1);
        this->declare_parameter<double>("wheel_distance",0.5);
        this->declare_parameter<double>("cmd_vel_timeout",0.5);
        this->declare_parameter<double>("cmd_vel_timer_frequency",10.0);
        this->declare_parameter<double>("reduction_ratio",90.0,onlyread_descriptor);//减速比
        this->declare_parameter<int>("encoder_resolution",0,onlyread_descriptor);//44
        this->declare_parameter<int>("max_velocity",100,onlyread_descriptor);//电机速度的最值,单位为 编码器计数/pid周期
        this->declare_parameter<int>("kp", 300);
        this->declare_parameter<int>("ki", 0);
        this->declare_parameter<int>("kd", 200);
        this->declare_parameter<std::string>("voltage_topic", "/battery_voltage");
        this->declare_parameter<std::string>("encoder_topic", "/encoder_data");
        //获取参数值
        this->get_parameter("port", port_);
        this->get_parameter("baud_rate", baud_rate_);
        this->get_parameter("data_bits", data_bits_);
        this->get_parameter("odom_frame", odom_frame_);
        this->get_parameter("odom_topic", odom_topic_);
        this->get_parameter("imu_frame", imu_frame_);
        this->get_parameter("imu_topic", imu_topic_);
        this->get_parameter("cmd_vel_topic", cmd_vel_topic_);
        this->get_parameter("wheel_diameter", wheel_diameter_);
        this->get_parameter("wheel_distance", wheel_distance_);
        this->get_parameter("cmd_vel_timeout", cmd_vel_timeout_);
        this->get_parameter("cmd_vel_timer_frequency", cmd_control_rate_);//定时器频率
        this->get_parameter("reduction_ratio", reduction_ratio_);
        this->get_parameter("encoder_resolution", encoder_resolution_);
        this->get_parameter("max_velocity", max_velocity_);
        this->get_parameter("kp", kp_);
        this->get_parameter("ki", ki_);
        this->get_parameter("kd", kd_);
        this->get_parameter("voltage_topic", voltage_topic_);
        this->get_parameter("encoder_topic", encoder_topic_);
        //实例化串口通信对象
        // serial_port_ = std::make_shared<my_serial::SerialPortComm>("/dev/mycar",115200,8);
        serial_port_ = std::make_shared<my_serial::SerialPortComm>(port_, baud_rate_, data_bits_);
        RCLCPP_INFO(
            this->get_logger(),
            "串口参数: port=%s, baud_rate=%d, data_bits=%d",
            port_.c_str(),
            baud_rate_,
            data_bits_);
        RCLCPP_INFO(this->get_logger(), "串口通信对象创建成功");

        //写入pid
        pid_set(kp_, ki_, kd_);

        //注册参数动态回调,用于运行时调整PID
        parameter_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&MyCarDriver::onParametersSet, this, _1));

        //创建订阅速度指令对象
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_topic_, 10, std::bind(&MyCarDriver::cmdVelCallback, this, _1));
        /* 
            在定时器中记录最新的速度指令对应的时间戳,如果超过一定时间没有收到新的速度指令
            则认为速度指令过期,需要将小车停止
        */
        cmd_vel_stop_ = std::make_shared<bool>(false);//初始化为false,表示没有发送停止指令
        //创建定时器对象,周期为100ms
        cmd_vel_timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / cmd_control_rate_)), std::bind(&MyCarDriver::cmdVelTimerCallback, this));
        cmd_msg_ = std::make_shared<geometry_msgs::msg::Twist>();//初始化速度指令消息对象

        //电压发布对象
        voltage_pub_ = this->create_publisher<std_msgs::msg::UInt16>(voltage_topic_, 10);
        encoder_pub_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>(encoder_topic_, 10);
        //里程计数据发布对象
        // odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);

        //判断参数不合法则直接退出
        if(cmd_control_rate_ <= 0 || cmd_vel_timeout_ <= 0 || wheel_diameter_ <= 0 || wheel_distance_ <= 0 || reduction_ratio_ <= 0 || encoder_resolution_ <= 0){
            RCLCPP_ERROR(this->get_logger(), "参数不合法,请检查参数配置");
            rclcpp::shutdown();
            return;
        }

        //启动单独串口读取线程(只允许一个线程读取串口)
        startThread();
    }

    ~MyCarDriver(){//析构函数,释放资源
        flag_ = false;//停止多线程循环
        msg_queue_cv_.notify_all();//唤醒等待中的分发线程
        serial_port_->stop_motor();//停止电机运动
    }

private:
    std::shared_ptr<my_serial::SerialPortComm> serial_port_;//串口通信对象指针
    std::string port_;//串口名称
    int baud_rate_,data_bits_;//波特率和数据位
    std::string odom_frame_, odom_topic_;//里程计坐标系和话题名称
    std::string imu_frame_, imu_topic_;//imu坐标系和话题名称
    std::string cmd_vel_topic_;//速度指令话题名称
    double wheel_diameter_;//车轮直径
    double wheel_distance_;//车轮间距
    double cmd_vel_timeout_;//速度指令超时时间
    double reduction_ratio_;//减速比
    int encoder_resolution_;//编码器分辨率
    int max_velocity_;//电机速度的最值,单位为转/s
    int kp_, ki_, kd_;//PID参数
    std::string voltage_topic_;//电压话题名称
    std::string encoder_topic_;//编码器数据话题名称

    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> cmd_vel_sub_;//速度指令订阅者
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);//速度指令消息处理
    
    std::shared_ptr<rclcpp::TimerBase> cmd_vel_timer_;//速度指令定时器
    double cmd_control_rate_;//定时器频率,单位为Hz
    void cmdVelTimerCallback();//速度指令定时器回调函数,用于检查速度指令是否过期

    rclcpp::Time last_cmd_vel_time_;//上次收到速度指令的时间戳
    std::shared_ptr<bool> cmd_vel_stop_;//通过设置共享指针并加锁防止定时器和回调同时修改cmd_vel_stop_变量导致的竞态条件
    std::mutex cmd_vel_mutex_;//保护cmd_vel_stop_变量的互斥锁
    std::shared_ptr<geometry_msgs::msg::Twist> cmd_msg_;//收到的速度指令

    std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> parameter_callback_handle_;//参数回调句柄
    rcl_interfaces::msg::SetParametersResult onParametersSet(const std::vector<rclcpp::Parameter> &parameters);//动态参数回调

    void pid_set(double kp, double ki, double kd){
        //设置pid参数
        serial_port_->write_pid(kp, ki, kd);
    }

    //创建线程的函数
    void startThread(){
        //一个读串口线程 + 一个分发线程
        std::thread(std::bind(&MyCarDriver::getMessage,this)).detach();
        std::thread(std::bind(&MyCarDriver::dispatchMessage,this)).detach();
    }
    //在子线程中读取数据
    void getMessage();
    //在子线程中分发数据
    void dispatchMessage();
    //解析电池电压数据并发布
    void publishBatteryVoltage(std::shared_ptr<my_serial::Message> msg);
    //解析编码器数据并发布
    void publishEncoderData(std::shared_ptr<my_serial::Message> msg);


    //主线程状态标记
    std::atomic<bool> flag_;

    std::mutex msg_queue_mutex_;
    std::condition_variable msg_queue_cv_;
    std::deque<std::shared_ptr<my_serial::Message>> msg_queue_;
    static constexpr std::size_t max_msg_queue_size_ = 100;
    
    //创建电压发布对象 电压单位mv
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr voltage_pub_;
    //里程计数据发布对象
    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr encoder_pub_;


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

//速度指令消息处理
void MyCarDriver::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg){
    std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
    last_cmd_vel_time_ = this->now();//更新上次收到速度指令的时间戳
    *cmd_vel_stop_ = false;//标记没有发送停止指令
    *cmd_msg_ = *msg;//更新速度指令消息对象
    //将速度指令(m/s)转换成电机可以用的参数
    //根据距离 = 2PI * 半径 * 转速
    //线速度 / 2PI / 半径 = 转速
    // double linear_vel = msg->linear.x;//线速度
    // double angular_vel = msg->angular.z;//角速度
    //先计算线速度对应的转速
    // double wheel_speed = linear_vel / (2 * M_PI * wheel_radius_);//转速,单位为转/s
    //再根据角速度计算左右轮的转速差

}

void MyCarDriver::cmdVelTimerCallback(){
    std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
    if(*cmd_vel_stop_){
        return;//如果已经发送了停止指令,则不需要再次发送
    }
    //检查速度指令是否过期,如果超过cmd_vel_timeout_没有收到新的速度指令,则认为过期,需要将小车停止
    if(this->now().seconds() - last_cmd_vel_time_.seconds() >= cmd_vel_timeout_){
        // RCLCPP_WARN(this->get_logger(), "速度指令过期,发送停止指令");
        //发送停止指令
        serial_port_->stop_motor();
        *cmd_vel_stop_ = true;//标记已经发送了停止指令
        return;
    }

    //实现速度转换的核心实现
    //将速度指令(m/s)转换成电机可以用的参数
    double linear_vel = cmd_msg_->linear.x;//线速度
    double angular_vel = cmd_msg_->angular.z;//角速度
    if(std::abs(linear_vel) < 1e-6 && std::abs(angular_vel) < 1e-6){
        //如果线速度和角速度都接近于0,则直接发送停止指令
        serial_port_->stop_motor();
        *cmd_vel_stop_ = true;//标记已经发送了停止指令
        return;
    }

    //将速度分别转换成左右轮的速度(m/s)

    /* 
        当线速度和角速度同时存在时
        先计算线速度对应的转速,再根据(角速度 * 半径)计算左右轮的转速差
        角速度为正,逆时针,向左转,左轮转速减小,右轮转速增大
        角速度为负,顺时针,向右转,左轮转速增大,右轮转速减小
        减一半加一半
        差速控制函数(左边电机,右边电机)
    */
    //将 m/s 转换成 转/s
    double linear_rand = linear_vel / (M_PI * wheel_diameter_);//线速度对应的转速,单位为转/s
    double angular_rand = angular_vel * wheel_distance_ / 2.0 / (M_PI * wheel_diameter_);//角速度对应的转速差,单位为转/s
    //如果角速度接近于0,则认为是纯线速度,直接将角速度置零
    if(std::abs(angular_vel) < 1e-6){
        angular_rand = 0.0;
    }
    //如果线速度接近于0,则认为是纯角速度,直接将线速度置零
    if(std::abs(linear_vel) < 1e-6){
        linear_rand = 0.0;
    }
    double left_wheel_rand = linear_rand - angular_rand;//左轮转速,单位为转/s
    double right_wheel_rand = linear_rand + angular_rand;//右轮转速,单位为转/s

    //将 转/s 转换成 编码器计数/s
    //编码器计数 = 转速 * 编码器每转的计数值(与编码器分辨率有关,脉冲数*4 = 分辨率) * 减速比
    left_wheel_rand = left_wheel_rand * (reduction_ratio_ * encoder_resolution_);//转速乘以编码器每转的计数值,单位为编码器计数/s
    right_wheel_rand = right_wheel_rand * (reduction_ratio_ * encoder_resolution_);//转速乘以编码器每转的计数值,单位为编码器计数/s

    //获取pid频率
    static int pid_rate = serial_port_->get_pid_rate();
    if (pid_rate == 0)
    {
        RCLCPP_ERROR(this->get_logger(), "PID频率为0,无法控制电机速度");
        return;
    }
    
    //将编码器计数/s 转换成 编码器计数/pid周期 (40ms 25hz)
    left_wheel_rand = left_wheel_rand / pid_rate;//单位为编码器计数/pid周期
    right_wheel_rand = right_wheel_rand / pid_rate;//单位为编码器计数/pid周期

    /* 
        bug 描述: 当电机速度较快时,左转弯实现不了,只能前进
        修改:
            设置电机速度的最值 v_max
            获取左右电机速度,并且取出最大值, max
            按照 max/v_max 的比例缩放左右电机速度,保证最大值不超过 v_max
            如果比例>=1.0 则 v_left = v_left / k, v_right = v_right / k
    */
    
    double scale = std::max(std::max(left_wheel_rand, right_wheel_rand) / max_velocity_, 1.0);
    left_wheel_rand = left_wheel_rand / scale;
    right_wheel_rand = right_wheel_rand / scale;

    //写出电机速度指令
    serial_port_->write_diff_drive_control(static_cast<short>(left_wheel_rand), static_cast<short>(right_wheel_rand));

}


rcl_interfaces::msg::SetParametersResult MyCarDriver::onParametersSet(const std::vector<rclcpp::Parameter> &parameters){
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;//默认成功

    int new_kp = kp_;
    int new_ki = ki_;
    int new_kd = kd_;
    bool has_pid_change = false;

    for(const auto &param : parameters){
        const auto &name = param.get_name();
        if(name != "kp" && name != "ki" && name != "kd"){
            continue;
        }

        if(param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER){
            result.successful = false;
            result.reason = name + " 必须是整数类型";
            return result;
        }

        const auto value = param.as_int();
        if(value < 0 || value > std::numeric_limits<short>::max()){
            result.successful = false;
            result.reason = name + " 超出范围(0~32767)";
            return result;
        }

        has_pid_change = true;
        if(name == "kp"){
            new_kp = static_cast<int>(value);
        }else if(name == "ki"){
            new_ki = static_cast<int>(value);
        }else{
            new_kd = static_cast<int>(value);
        }
    }

    if(!has_pid_change){
        return result;
    }

    kp_ = new_kp;
    ki_ = new_ki;
    kd_ = new_kd;
    pid_set(kp_, ki_, kd_);

    RCLCPP_INFO(this->get_logger(), "PID参数动态更新成功: kp=%d, ki=%d, kd=%d", kp_, ki_, kd_);
    return result;
}


//在子线程中读取数据
void MyCarDriver::getMessage(){
    while(rclcpp::ok() && flag_){
        //只负责读取串口数据并入队,不做业务处理
        auto msg = serial_port_->read_message();
        if(msg == nullptr){//在头文件中我有设置,不会出现空指针现像,但是为了保险起见,我还是加上这个判断
            continue;//读取数据失败,继续下一次循环
        }

        {
            std::lock_guard<std::mutex> lock(msg_queue_mutex_);
            if(msg_queue_.size() >= max_msg_queue_size_){
                msg_queue_.pop_front();//队列满时丢弃最旧数据,保证读取线程不被阻塞
            }
            msg_queue_.push_back(msg);
        }
        msg_queue_cv_.notify_one();
    }
}

void MyCarDriver::dispatchMessage(){
    while(rclcpp::ok() && flag_){
        std::shared_ptr<my_serial::Message> msg;
        {
            std::unique_lock<std::mutex> lock(msg_queue_mutex_);
            msg_queue_cv_.wait(lock, [this](){
                return !msg_queue_.empty() || !flag_;
            });

            if(!flag_ && msg_queue_.empty()){
                return;
            }

            msg = msg_queue_.front();
            msg_queue_.pop_front();
        }

        if(msg == nullptr){
            continue;
        }

        switch (msg->function_code)
        {
        case my_serial::FunctionCode::VOLTAGE://电压数据
            publishBatteryVoltage(msg);
            break;
        case my_serial::FunctionCode::WHEEL_ENCODER://编码器数据(轮速)
            publishEncoderData(msg);
            break;
        default:
            break;
        }
    }
}


//解析电池电压数据并发布
/**
 * @brief 循环读取电池电压数据并发布。
 *
 * 此函数会在ROS2节点运行且flag_为真时持续循环，从串口读取电池电压数据。
 * 读取的数据通过合并高低字节转换为16位整数（单位为毫伏），并以ROS2消息的形式发布。
 * 若读取失败（返回空指针），则跳过本次循环，继续尝试读取。
 *
 * 注意：头文件中已保证不会出现空指针，但为保险起见仍做了空指针判断。
 */
void MyCarDriver::publishBatteryVoltage(std::shared_ptr<my_serial::Message> msg){
    //处理电压数据
    std_msgs::msg::UInt16 vol;
    vol.data = ((msg->data[0] << 8) & 0xff00) | (msg->data[1] & 0x00ff);//将高字节和低字节合并成一个16位的整数,单位为mv
    //发布电压数据
    voltage_pub_->publish(vol);
    rclcpp::Rate(10).sleep();//10hz
}

//解析编码器数据并发布
void MyCarDriver::publishEncoderData(std::shared_ptr<my_serial::Message> msg){//8数据位,每个轮子4字节
    //处理编码器数据
    std_msgs::msg::UInt16MultiArray encoder_data;
    // encoder_data.data.resize(msg->data.size());
    // int temp = 0;
    // for(size_t i = 0; i < 8; ++i){
    //     i % 2 ==0 ? (temp |= (msg->data[i] & 0x00ff)) : (temp = (msg->data[i] << 8) & 0xff00);
    // }
    for(size_t i = 0; i < 4; ++i){
        encoder_data.data.push_back(((msg->data[i*2] << 8) & 0xff00) | (msg->data[i*2+1] & 0x00ff));
    }
    encoder_pub_->publish(encoder_data);
    rclcpp::Rate(10).sleep();//10hz
}