#include "ros2_stm32_bridge/ros2_stm32_driver_node.hpp"

#include "iostream"

namespace ros2_stm32 {
namespace driver {

MiniDriver::MiniDriver() : Node("mini_driver"), parse_flag_(false), start_flag_(true) {

  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.read_only = true;

  this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0",desc);
  this->declare_parameter<int>("baud_rate", 115200,desc);
  this->declare_parameter<std::string>("odom_frame", "odom",desc);
//   this->declare_parameter<std::string>("base_frame", "base_link",desc);
  this->declare_parameter<std::string>("base_frame", "base_footprint",desc);
  this->declare_parameter<int>("control_rate", 20,desc);
  this->declare_parameter<double>("maximum_encoding", 100.0);
  this->declare_parameter<double>("encoder_resolution", 44.0,desc);
  this->declare_parameter<double>("reduction_ratio", 90.0,desc);
  this->declare_parameter<double>("model_param_cw", 0.216);
  this->declare_parameter<double>("model_param_acw", 0.216);
  this->declare_parameter<double>("wheel_diameter", 0.065);
  this->declare_parameter<int>("kp", 25);
  this->declare_parameter<int>("ki", 0);
  this->declare_parameter<int>("kd", 30);

  //自定义参数



  this->get_parameter("port_name", port_name_);
  this->get_parameter("baud_rate", baud_rate_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("control_rate", control_rate_);
  control_rate_timer_ = std::make_unique<rclcpp::Rate>(control_rate_);
  // this->get_parameter("maximum_encoding", maximum_encoding_);
  this->get_parameter("encoder_resolution", encoder_resolution_);
  this->get_parameter("reduction_ratio", reduction_ratio_);
  // this->get_parameter("model_param_cw", model_param_cw_);
  // this->get_parameter("model_param_acw", model_param_acw_);
  // this->get_parameter("wheel_diameter", wheel_diameter_);
  // pulse_per_cycle_ = reduction_ratio_ * encoder_resolution_ / (M_PI * wheel_diameter_ * pid_rate_);
  pid_rate_ = 25.0;
  last_twist_time_ = this->get_clock()->now();

  voltage_publisher_ = this->create_publisher<std_msgs::msg::UInt16>("voltage", 10);
  wheel_encode_publisher_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("wheel_encode", 10);
  imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
  odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 100);

  twist_subscribe_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&MiniDriver::TwistHandleCallback, this, std::placeholders::_1));

  send_timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000 / control_rate_)),
                                        std::bind(&MiniDriver::SendTimerCallback, this));
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  //定时器发布tf变换
  tf_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),//1000 / 50 ms = 20Hz
    std::bind(&MiniDriver::PublishTfOdomTimer, this)
  );

  Run();
}


MiniDriver::~MiniDriver() {
  mutex_.lock();
  parse_flag_ = false;
  SpeedCommand(0,0);
  if (port_) {
    port_->cancel();
    port_->close();
    port_.reset();
  }
  io_service_.stop();
  io_service_.reset();
  mutex_.unlock();
}

void MiniDriver::PublishTfOdomTimer(){
    double x,y,yaw,v,w;
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    x = last_x_; y = last_y_; yaw = last_yaw_;
    v = last_v_; w = last_w_;
  }

  auto stamp = this->get_clock()->now();

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = odom_frame_;
  tf.child_frame_id = base_frame_;
  tf.transform.translation.x = x;
  tf.transform.translation.y = y;
  tf.transform.translation.z = 0.0;
  tf2::Quaternion q; q.setRPY(0,0,yaw);
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf.transform.rotation.w = q.w();
  tf_broadcaster_->sendTransform(tf);

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_frame_;
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();
  odom.twist.twist.linear.x = v;
  odom.twist.twist.angular.z = w;
  odom_publisher_->publish(odom);
}

bool MiniDriver::Init() {
  if (port_) {
    RCLCPP_ERROR(this->get_logger(), "error : port is already opened...");
    return false;
  }

  port_ = serial_port_ptr(new boost::asio::serial_port(io_service_));
  port_->open(port_name_, ec_);
  if (ec_) {
    RCLCPP_INFO_STREAM(this->get_logger(), "error : port_->open() failed...port_name="
                                               << port_name_ << ", e=" << ec_.message().c_str() << "\n");
    return false;
  }
  // option settings...
  port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
  port_->set_option(boost::asio::serial_port_base::character_size(8));
  port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
  port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
  port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
  return true;
}

void MiniDriver::ParseMessage() {
  uint8_t check_num, payload[8], buffer_data[255];
  MessageType msg_type;
  ParseStatus status = HEADER;
  parse_flag_ = true;
  while (rclcpp::ok() && parse_flag_) {
    
    switch (status) {
      case HEADER:
        boost::asio::read(*port_.get(), boost::asio::buffer(&buffer_data[0], 1), ec_);
        if (buffer_data[0] == 0xfc) {
          check_num = 0xfc;
          status = TYPE; 
        }
        break;

      case TYPE:
        boost::asio::read(*port_.get(), boost::asio::buffer(&buffer_data[0], 1), ec_);
        if (buffer_data[0] >= 1 && buffer_data[0] <= 5) {
          msg_type = MessageType(buffer_data[0]);
          check_num ^= buffer_data[0];
          status = DATA;
        } else {
          status = HEADER;
        }
        break;

      case DATA:
        boost::asio::read(*port_.get(), boost::asio::buffer(&payload[0], 8), ec_);
        for (int i = 0; i < 8; i++) {
          check_num ^= payload[i];
        }
        status = CHECKSUM;
        break;

      case CHECKSUM:
        boost::asio::read(*port_.get(), boost::asio::buffer(&buffer_data[0], 1), ec_);
        if (buffer_data[0] == check_num) {
          status = END;
        } else {
          status = HEADER;
        }
        break;

      case END:
        boost::asio::read(*port_.get(), boost::asio::buffer(&buffer_data[0], 1), ec_);
        if (buffer_data[0] == 0xdf) {
          DistributeMessage(msg_type, payload);
        }
        status = HEADER;
        // control_rate_timer_->sleep();
        break;
      default:
        break;
    }
  }
}

void MiniDriver::DistributeMessage(MessageType type, uint8_t* payload) {
  if (type == VOLTAGE) {
    std_msgs::msg::UInt16 voltage_msg;
    voltage_msg.data = (payload[0] << 8 & 0xff00) | (payload[1] & 0x00ff);
    voltage_publisher_->publish(voltage_msg);
  } else if (type == WHEEL_ENCODE) {
    std_msgs::msg::UInt16MultiArray wheel_encode_msg;
    for (int i = 0; i < 4; i++) {
      wheel_encode_msg.data.push_back((payload[2 * i] << 8 & 0xff00) | (payload[2 * i + 1] & 0x00ff));
    }
    wheel_encode_publisher_->publish(wheel_encode_msg);
    HandleEncodeMessage(wheel_encode_msg.data.at(0), wheel_encode_msg.data.at(1));
  } else if (type == ANGULAR_VELOCITY) {
    imu_msg_.angular_velocity.x = (int16_t)((payload[0] << 8 & 0xff00) | (payload[1] & 0x00ff)) * 0.1;
    imu_msg_.angular_velocity.y = (int16_t)((payload[2] << 8 & 0xff00) | (payload[3] & 0x00ff)) * 0.1;
    imu_msg_.angular_velocity.z = (int16_t)((payload[4] << 8 & 0xff00) | (payload[5] & 0x00ff)) * 0.1;
    imu_msg_flag_[0] = true;
    CheckAndPublishImu();
  } else if (type == ACCELERATION) {
    imu_msg_.linear_acceleration.x = (int16_t)((payload[0] << 8 & 0xff00) | (payload[1] & 0x00ff)) * 0.1;
    imu_msg_.linear_acceleration.y = (int16_t)((payload[2] << 8 & 0xff00) | (payload[3] & 0x00ff)) * 0.1;
    imu_msg_.linear_acceleration.z = (int16_t)((payload[4] << 8 & 0xff00) | (payload[5] & 0x00ff)) * 0.1;
    imu_msg_flag_[1] = true;
    CheckAndPublishImu();
  } else if (type == ORIENTATION) {
    tf2::Quaternion quaternion;
    quaternion.setRPY((int16_t)((payload[0] << 8 & 0xff00) | (payload[1] & 0x00ff)) * 0.001744444,
                      -(int16_t)((payload[2] << 8 & 0xff00) | (payload[3] & 0x00ff)) * 0.001744444,
                      (int16_t)((payload[4] << 8 & 0xff00) | (payload[5] & 0x00ff)) * 0.001744444);
    imu_msg_.orientation.x = quaternion.getX();
    imu_msg_.orientation.y = quaternion.getY();
    imu_msg_.orientation.z = quaternion.getZ();
    imu_msg_.orientation.w = quaternion.getW();
    imu_msg_flag_[2] = true;
    CheckAndPublishImu();
  }
}

void MiniDriver::CheckAndPublishImu() {
  if (imu_msg_flag_[0] && imu_msg_flag_[1] && imu_msg_flag_[2]) {
    imu_msg_.header.frame_id = "imu_link";
    imu_msg_.header.stamp = this->get_clock()->now();
    imu_publisher_->publish(imu_msg_);
    std::fill_n(std::begin(imu_msg_flag_), 3, false);
  }
}

int MiniDriver::CalculateDelta(int& current_encode, int receive_encode) {
  int delta;
  if (receive_encode > current_encode) {
    delta = (receive_encode - current_encode) < (current_encode - receive_encode + 65535)
                ? (receive_encode - current_encode)
                : (receive_encode - current_encode - 65535);
  } else {
    delta = (current_encode - receive_encode) < (receive_encode - current_encode + 65535)
                ? (receive_encode - current_encode)
                : (receive_encode - current_encode + 65535);
  }
  current_encode = receive_encode;
  return delta;
}

void MiniDriver::HandleEncodeMessage(short left_encode, short right_encode) {
  now_ = this->get_clock()->now();
  if (start_flag_) {
    accumulation_x_ = 0.0;
    accumulation_y_ = 0.0;
    accumulation_th_ = 0.0;
    current_left_encode_ = left_encode;
    current_right_encode_ = right_encode;
    last_time_ = now_;
    start_flag_ = false;
    return;
  }
  int delta_left = CalculateDelta(current_left_encode_, left_encode);
  int delta_right = CalculateDelta(current_right_encode_, right_encode);
  delta_time_ = (now_ - last_time_).seconds();

  double min_dt = 1.0 / std::max(1, control_rate_);
if (delta_time_ < min_dt) {
  return;
}
//   if (delta_time_ >= 0.02) {
    double model_param;
    this->get_parameter("model_param_cw", model_param_cw_);
    this->get_parameter("model_param_acw", model_param_acw_);
    if (delta_right <= delta_left) {
      model_param = model_param_cw_;
    } else {
      model_param = model_param_acw_;
    }
    this->get_parameter("wheel_diameter", wheel_diameter_);
    pulse_per_cycle_ = reduction_ratio_ * encoder_resolution_ / (M_PI * wheel_diameter_ * pid_rate_);
    double delta_theta = (delta_right - delta_left) / (pulse_per_cycle_ * pid_rate_ * model_param);
    double v_theta = delta_theta / delta_time_;
    
    double delta_dis = (delta_right + delta_left) / (pulse_per_cycle_ * pid_rate_ * 2.0);
    double v_dis = delta_dis / delta_time_;
    double delta_x, delta_y;
    if (delta_theta == 0) {
      delta_x = delta_dis;
      delta_y = 0.0;
    } else {
      delta_x = delta_dis * (sin(delta_theta) / delta_theta);
      delta_y = delta_dis * ((1 - cos(delta_theta)) / delta_theta);
    }

    accumulation_x_ += (cos(accumulation_th_) * delta_x - sin(accumulation_th_) * delta_y);
    accumulation_y_ += (sin(accumulation_th_) * delta_x + cos(accumulation_th_) * delta_y);
    accumulation_th_ += delta_theta;

//   }

    {
  std::lock_guard<std::mutex> lk(state_mtx_);
  last_x_ = accumulation_x_;
  last_y_ = accumulation_y_;
  last_yaw_ = accumulation_th_;
  last_v_ = v_dis;
  last_w_ = v_theta;
}

  last_time_ = now_;
}

void MiniDriver::TwistHandleCallback(const geometry_msgs::msg::Twist& msg) {
  twist_mutex_.lock();
  last_twist_time_ = this->get_clock()->now();
  current_twist_ = msg;
//   Pid();
  twist_mutex_.unlock();
}

void MiniDriver::SpeedCommand(short left,short right){
//   uint8_t data[12] = {0xfc, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xdf};
//   data[2] = data[6] = (left >> 8) & 0xff;
//   data[3] = data[7] = left & 0xff;
//   data[4] = data[8] = (right >> 8) & 0xff;
//   data[5] = data[9] = right & 0xff;
//   for (int i = 0; i < 10; i++) {
//     data[10] ^= data[i];
//   }
//   boost::asio::write(*port_.get(), boost::asio::buffer(data, 12), ec_);

std::lock_guard<std::mutex> lk(serial_tx_mtx_);

  if (!port_ || !port_->is_open()) {
    RCLCPP_ERROR(this->get_logger(), "SpeedCommand: serial not open");
    return;
  }

  uint8_t data[12] = {0xfc, 0x06, 0,0, 0,0, 0,0, 0,0, 0, 0xdf};

  // 你现在用的四轮重复格式（按你最新的帧）
  data[2] = data[6] = (left  >> 8) & 0xff;
  data[3] = data[7] =  left        & 0xff;
  data[4] = data[8] = (right >> 8) & 0xff;
  data[5] = data[9] =  right       & 0xff;

  uint8_t chk = 0;
  for (int i = 0; i < 10; ++i) chk ^= data[i];
  data[10] = chk;

  boost::system::error_code ec;
  size_t n = boost::asio::write(*port_, boost::asio::buffer(data, 12), ec);

  if (ec || n != 12) {
    // 这里才是真写失败（再也不会被读线程污染）
    RCLCPP_ERROR(this->get_logger(),
      "SERIAL WRITE FAIL: n=%zu ec=%d(%s)",
      n, ec.value(), ec.message().c_str());
  }
}

void MiniDriver::SendTimerCallback() {

// 0) 串口是否可用
  if (!port_ || !port_->is_open()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "serial not open");
    return;
  }

  // 1) 线程安全读取 twist 与时间
  geometry_msgs::msg::Twist twist;
  rclcpp::Time last_time;
  {
    std::lock_guard<std::mutex> lk(twist_mutex_);
    twist = current_twist_;
    last_time = last_twist_time_;
  }

  double linear_speed = 0.0, angular_speed = 0.0;
  if ((this->get_clock()->now() - last_time).seconds() <= 1.0) {
    linear_speed = twist.linear.x;
    angular_speed = twist.angular.z;
  }

  // 2) 每次确保关键参数已加载（别依赖“某处算过一次”）
  this->get_parameter("model_param_cw", model_param_cw_);
  this->get_parameter("model_param_acw", model_param_acw_);
  this->get_parameter("wheel_diameter", wheel_diameter_);
  this->get_parameter("encoder_resolution", encoder_resolution_);
  this->get_parameter("reduction_ratio", reduction_ratio_);
  this->get_parameter("maximum_encoding", maximum_encoding_);

  // 3) 重新计算 pulse_per_cycle_（防止未初始化/为0）
  // 注意：你原来用 pid_rate_，这里仍按你的协议逻辑保留
  pulse_per_cycle_ = reduction_ratio_ * encoder_resolution_ / (M_PI * wheel_diameter_ * pid_rate_);

  if (pulse_per_cycle_ <= 1e-9) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "pulse_per_cycle_ invalid: %.12f", pulse_per_cycle_);
    return;
  }

  double model_param = (angular_speed <= 0) ? model_param_cw_ : model_param_acw_;

  double left_d  = (linear_speed - model_param / 2.0 * angular_speed) * pulse_per_cycle_;
  double right_d = (linear_speed + model_param / 2.0 * angular_speed) * pulse_per_cycle_;

  double radio = std::max(std::max(std::abs(left_d), std::abs(right_d)) / maximum_encoding_, 1.0);

  short left  = static_cast<short>(left_d / radio);
  short right = static_cast<short>(right_d / radio);

  //调试信息
//   RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
//     "v=%.3f w=%.3f pulse=%.3f mp=%.3f left_d=%.2f right_d=%.2f -> L=%d R=%d",
//     linear_speed, angular_speed, pulse_per_cycle_, model_param, left_d, right_d, (int)left, (int)right
//   );

  SpeedCommand(left, right);
// SpeedCommand(80, 80);
// return;

//   double left_d, right_d, radio;
//   double model_param;
//   short left, right;

//   double linear_speed, angular_speed;
//   if ((this->get_clock()->now() - last_twist_time_).seconds() <= 1.0) {
//     linear_speed = current_twist_.linear.x;
//     angular_speed = current_twist_.angular.z;
//   } else {
//     linear_speed = 0;
//     angular_speed = 0;
//   }
//   if (angular_speed <= 0) {
//     model_param = model_param_cw_;
//   } else {
//     model_param = model_param_acw_;
//   }

//   left_d = (linear_speed - model_param / 2 * angular_speed) * pulse_per_cycle_;
//   right_d = (linear_speed + model_param / 2 * angular_speed) * pulse_per_cycle_;

//   this->get_parameter("maximum_encoding", maximum_encoding_);
//   radio = std::max(std::max(std::abs(left_d), std::abs(right_d)) / maximum_encoding_, 1.0);

//   left = static_cast<short>(left_d / radio);
//   right = static_cast<short>(right_d / radio);

//   SpeedCommand(left,right);
  
}

void MiniDriver::Pid(){
//   int64_t kp,ki,kd;
//   this->get_parameter("kp",kp);
//   this->get_parameter("ki",ki); 
//   this->get_parameter("kd",kd); 
//   uint8_t data[12] = {0xfc, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xdf};
//   data[2] = (kp >> 8) & 0xff;
//   data[3] = kp & 0xff;
//   data[4] = (ki >> 8) & 0xff;
//   data[5] = ki & 0xff;
//   data[6] = (kd >> 8) & 0xff;
//   data[7] = kd & 0xff;
//   for (int i = 0; i < 10; i++) {
//     data[10] ^= data[i];
//   }

//   boost::asio::write(*port_.get(), boost::asio::buffer(data, 12), ec_);

 std::lock_guard<std::mutex> lk(serial_tx_mtx_);

  if (!port_ || !port_->is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Pid: serial not open");
    return;
  }

  int64_t kp, ki, kd;
  this->get_parameter("kp", kp);
  this->get_parameter("ki", ki);
  this->get_parameter("kd", kd);

  uint8_t data[12] = {0xfc, 0x07, 0,0, 0,0, 0,0, 0,0, 0, 0xdf};
  data[2] = (kp >> 8) & 0xff; data[3] = kp & 0xff;
  data[4] = (ki >> 8) & 0xff; data[5] = ki & 0xff;
  data[6] = (kd >> 8) & 0xff; data[7] = kd & 0xff;

  uint8_t chk = 0;
  for (int i = 0; i < 10; ++i) chk ^= data[i];
  data[10] = chk;

  boost::system::error_code ec;
  size_t n = boost::asio::write(*port_, boost::asio::buffer(data, 12), ec);

  if (ec || n != 12) {
    RCLCPP_ERROR(this->get_logger(),
      "SERIAL PID WRITE FAIL: n=%zu ec=%d(%s)",
      n, ec.value(), ec.message().c_str());
  }
}
void MiniDriver::Run() {
  if (Init()) {
    Pid();
    std::thread parse_thread(&MiniDriver::ParseMessage, this);
    parse_thread.detach();
  }
}

}  // namespace driver
}  // namespace ros2_stm32

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ros2_stm32::driver::MiniDriver>());
  rclcpp::shutdown();
  return 0;
}
