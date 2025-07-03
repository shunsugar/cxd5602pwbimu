#ifndef SPRESENSE_IMU_NODE_HPP
#define SPRESENSE_IMU_NODE_HPP

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
  void controlLoop();
  void openSerial(const std::string & port, const int & baudrate, const float & time_out);

  std::string port_;
  int baudrate_;
  float time_out_;

private:
  void processData(const uint8_t & raw_data);
  void publishMsg(const std::vector<float> & acceleration,
                  const std::vector<float> & angular_velocity,
                  const std::vector<float> & quaternion);

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::Imu imu_msg_;
  serial::Serial imu_serial_;
  std::string frame_id_;
  std::string imu_topic_;
  std::vector<uint8_t> buff_;
  std::vector<float> acceleration_;
  std::vector<float> angular_velocity_;
  std::vector<float> quaternion_;

  const size_t BUFFER_SIZE = 22;
  const uint8_t HEADER_BYTE = 0x55;
  const float ACC_SCALE = 9.80665f / 8192.0f;
  const float GYR_SCALE = (M_PI / 180.0f) / 65.536f;
  const float QUAT_SCALE = 1.0f / 10000.0f;
};

#endif // SPRESENSE_IMU_NODE_HPP
