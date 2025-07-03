#include "spresense_imu_node.hpp"

SpresenseImuNode::SpresenseImuNode() : rclcpp::Node("spresense_imu_node")
{
  this->declare_parameter<std::string>("port", "/dev/sensors/Spresense_IMU");
  this->declare_parameter<std::string>("frame_id", "imu_link");
  this->declare_parameter<std::string>("imu_topic", "/imu/Spresense");
  this->declare_parameter<float>("time_out", 10);
  this->declare_parameter<int>("baudrate", 115200);
  this->get_parameter("port", port_);
  this->get_parameter("frame_id", frame_id_);
  this->get_parameter("imu_topic", imu_topic_);
  this->get_parameter("time_out", time_out_);
  this->get_parameter("baudrate", baudrate_);

  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 10);
  timer_ = this->create_wall_timer(2ms, std::bind(&SpresenseImuNode::controlLoop, this));

  RCLCPP_INFO(this->get_logger(), "Initialized spresense_imu_node");
  openSerial(port_, baudrate_, time_out_);
}

SpresenseImuNode::~SpresenseImuNode()
{
  if (imu_serial_.isOpen()) {
    imu_serial_.close();
  }
}

void SpresenseImuNode::openSerial(const std::string & port, const int & baudrate, const float & time_out)
{
  RCLCPP_INFO(this->get_logger(), "Port: %s, baud: %d", port.c_str(), baudrate);
  serial::Timeout to = serial::Timeout::simpleTimeout(10);

  while (rclcpp::ok()) {
    try {
      // Attempt to open the serial port
      imu_serial_.setPort(port);
      imu_serial_.setBaudrate(baudrate);
      imu_serial_.setTimeout(to);
      imu_serial_.open();
      RCLCPP_INFO(this->get_logger(), "Serial port opened successfully");
      break;
    }
    catch (serial::IOException& e) {
      RCLCPP_WARN(this->get_logger(), "Serial port opening failure... Retrying");
      rclcpp::sleep_for(5s);
    }
  }
}

void SpresenseImuNode::controlLoop()
{
  try {
    if (imu_serial_.available()) {
      std::vector<uint8_t> buff_data;
      imu_serial_.read(buff_data, imu_serial_.available());
      for (const auto & byte : buff_data) {
        processData(byte);
      }
    }
  }
  catch (const std::exception & e) {
    RCLCPP_WARN(this->get_logger(), "IMU disconnect... Retrying");
    openSerial(port_, baudrate_, time_out_);
  }
}

void SpresenseImuNode::processData(const uint8_t & raw_data)
{
  buff_.push_back(raw_data);
  
  if (buff_[0] != HEADER_BYTE) {
    buff_.clear();
    return;
  }
  
  if (buff_.size() < BUFFER_SIZE) {
    return;
  }
  
  uint8_t checksum = 0;
  for (size_t i = 1; i < BUFFER_SIZE - 1; ++i) {
    checksum ^= buff_[i];
  }
  
  if (checksum != buff_[BUFFER_SIZE - 1]) {
    RCLCPP_WARN(this->get_logger(), "Checksum error");
    buff_.erase(buff_.begin());
    return;
  }
  
  int16_t data[10];
  for (int i = 0; i < 10; ++i) {
    data[i] = static_cast<int16_t>(buff_[2 * i + 2] << 8 | buff_[2 * i + 1]);
  }
  
  acceleration_ = {
    data[0] * ACC_SCALE,
    data[1] * ACC_SCALE,
    data[2] * ACC_SCALE
  };
  
  angular_velocity_ = {
    data[3] * GYR_SCALE,
    data[4] * GYR_SCALE,
    data[5] * GYR_SCALE
  };
  
  quaternion_ = {
    data[6] * QUAT_SCALE,
    data[7] * QUAT_SCALE,
    data[8] * QUAT_SCALE,
    data[9] * QUAT_SCALE
  };
  
  publishMsg(acceleration_, angular_velocity_, quaternion_);
  
  buff_.clear();
}

void SpresenseImuNode::publishMsg(const std::vector<float> & acceleration,
                          const std::vector<float> & angular_velocity,
                          const std::vector<float> & quaternion)
{
  imu_msg_.header.stamp = this->get_clock()->now();
  imu_msg_.header.frame_id = frame_id_;
  imu_msg_.linear_acceleration.x = acceleration[0];
  imu_msg_.linear_acceleration.y = acceleration[1];
  imu_msg_.linear_acceleration.z = acceleration[2];
  imu_msg_.angular_velocity.x = angular_velocity[0];
  imu_msg_.angular_velocity.y = angular_velocity[1];
  imu_msg_.angular_velocity.z = angular_velocity[2];
  imu_msg_.orientation.x = quaternion[0];
  imu_msg_.orientation.y = quaternion[1];
  imu_msg_.orientation.z = quaternion[2];
  imu_msg_.orientation.w = quaternion[3];
  
  imu_pub_->publish(imu_msg_);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SpresenseImuNode>();
  while (rclcpp::ok()) {
    node->controlLoop();
  }
  rclcpp::shutdown();
  return 0;
}
