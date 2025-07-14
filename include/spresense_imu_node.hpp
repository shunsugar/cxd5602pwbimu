#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <serial/serial.h>

class SpresenseImuNode : public rclcpp::Node
{
public:
  SpresenseImuNode();
  ~SpresenseImuNode();

private:
  /* ------ パラメータ ------- */
  std::string port_, frame_id_, imu_topic_;
  float  time_out_;
  int    baudrate_;

  /* ------ 定数 ------- */
  static constexpr uint8_t HEADER_BYTE  = 0x55;
  static constexpr size_t  PKT_SIZE     = 30;          // 0x55 + 32 + CRC
  static constexpr size_t  PAYLOAD_SIZE = 28;          // imu_raw32_t
  static constexpr size_t  CHECK_IDX    = 29;          // CRC
  static constexpr double  DEG2RAD      = M_PI / 180.0;

  /* ------ ハード／ROS リソース ------- */
  serial::Serial                       imu_serial_;
  rclcpp::TimerBase::SharedPtr         timer_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  sensor_msgs::msg::Imu                imu_msg_;

  /* ------ 受信バッファ ------- */
  std::vector<uint8_t> buf_;

  /* ------ 内部処理 ------- */
  void openSerial(const std::string&, const int&, const float&);
  void controlLoop();
  void processByte(uint8_t byte);
  void publishImu(const float acc[3], const float gyr[3]);

  /* ------ ヘルパ ------- */
  static uint8_t xor_checksum(const uint8_t* data, size_t len)
  {
    uint8_t c = 0; while (len--) c ^= *data++; return c;
  }
};

