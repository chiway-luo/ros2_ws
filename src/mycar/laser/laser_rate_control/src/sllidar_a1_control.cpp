#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

/*
    需求:将雷达发布的消息频率稳定在20hz
    流程:
        1.包含头文件
        2.初始化ros2客户端
        3.自定义节点类
            3-1
            3-2
            3-3
        4.调用spin函数,并传入节点对象指针
        5.释放资源
*/

using namespace std::chrono_literals; //使用时间命名空间
using namespace std::placeholders; //占位符命名空间
std::mutex mutex_; //互斥锁

class RateControl :public rclcpp::Node{
public:
    RateControl(std::string str1,std::string str2):Node(str1,str2){
        //定义参数
        this->declare_parameter<int>("rate",20);

        RCLCPP_INFO(this->get_logger(),"namesapce: %s node: %s 节点创建成功",str2.c_str(),str1.c_str());
        //创建雷达消息接收节点
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan_info",
            rclcpp::QoS(rclcpp::KeepLast(20)),
            std::bind(&RateControl::sub_scan,this,_1)
        );
        //创建定时器,周期发布雷达消息
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000/get_parameter("rate").as_int()), //20hz
            std::bind(&RateControl::pub_scan,this)
        );
        //创建雷达消息发布方
        laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/scan",
            rclcpp::QoS(rclcpp::KeepLast(20))
        );

    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    sensor_msgs::msg::LaserScan::SharedPtr laser_msg_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;

    //回调函数
    void sub_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg){
        //复制指针
        {
            std::lock_guard<std::mutex> lock(mutex_); //加锁
            laser_msg_ptr_ = msg;
        }
    }
    void pub_scan(){
        //创建雷达消息对象
        sensor_msgs::msg::LaserScan scan_msg;
        //获取雷达消息
        {
            std::lock_guard<std::mutex> lock(mutex_); //加锁
            if(laser_msg_ptr_ == nullptr)return; //如果指针为空,直接返回
            scan_msg = *laser_msg_ptr_;
        }
        //修改时间戳
        // scan_msg.header.stamp = this->now();
        //发布雷达消息
        laser_pub_->publish(scan_msg);
    }
};

int main(int argc, char * argv[])
{
    //初始化ros2客户端
    rclcpp::init(argc,argv);

    //调用spin函数,使用自定义类对象指针
    rclcpp::spin(std::make_shared<RateControl>("ratecontrol_node_cpp","my_car"));//node_name,namespace

    //释放资源
    rclcpp::shutdown();
    return 0;
}