#ifndef SPRESENSE_IMU_NODE_HPP_
#define SPRESENSE_IMU_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <serial/serial.h>

#include <chrono>

using namespace std::chrono_literals;

class SpresenseImuNode : public rclcpp::Node
{
public:
  SpresenseImuNode();
  ~SpresenseImuNode();

private:
  void openSerial();
  void publishMsg();
  
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  serial::Serial serial_;
  
  std::string port_;
  std::string frame_id_;
  std::string imu_topic_;
  float time_out_;
  int baudrate_;
}


#endif // SPRESENSE_IMU_NODE_HPP_

