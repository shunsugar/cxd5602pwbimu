/*
 * spresense_imu_bridge_serial.cpp
 *
 * ROS 2 C++ node that reads 44‑byte frames from a Spresense board over
 * UART using the header‑only **serial/serial.h** library, validates a
 * CRC‑16/CCITT checksum, and publishes the data as **sensor_msgs/Imu**
 * on the "/imu" topic.
 *
 * Frame layout (little‑endian):
 *   0xAA 0x55 | float q[4] | float gyro[3] | float accel[3] | CRC16
 *   └─2 B───┘   └────── 40 bytes payload ──────┘   └─2 B───┘
 * Total: 44 bytes per frame.
 *
 * Build excerpt (CMakeLists.txt):
 * --------------------------------------------------------------
 * find_package(ament_cmake REQUIRED)
 * find_package(rclcpp REQUIRED)
 * find_package(sensor_msgs REQUIRED)
 * find_package(serial REQUIRED)   # apt install ros-<distro>-serial
 *
 * add_executable(imu_bridge_serial src/spresense_imu_bridge_serial.cpp)
 * ament_target_dependencies(imu_bridge_serial rclcpp sensor_msgs serial)
 * install(TARGETS imu_bridge_serial DESTINATION lib/${PROJECT_NAME})
 * --------------------------------------------------------------
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <serial/serial.h>

#include <array>
#include <vector>
#include <algorithm>
#include <cstring>
#include <cstdint>
#include <chrono>

class SpresenseImuSerialNode : public rclcpp::Node
{
public:
  SpresenseImuSerialNode()
  : Node("spresense_imu_serial_node"), ser_()
  {
    /* ---------- Parameters ---------- */
    std::string port = declare_parameter<std::string>("port", "/dev/ttyUSB0");
    int baud         = declare_parameter<int>("baud_rate", 115200);
    frame_id_        = declare_parameter<std::string>("frame_id", "imu_link");

    /* ---------- Serial open ---------- */
    ser_.setPort(port);
    ser_.setBaudrate(static_cast<uint32_t>(baud));
    serial::Timeout to = serial::Timeout::simpleTimeout(20); // 20 ms
    ser_.setTimeout(to);
    ser_.open();

    /* ---------- ROS publisher ---------- */
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu", 10);

    /* ---------- Poll timer (1 ms) ---------- */
    timer_ = create_wall_timer(
      std::chrono::milliseconds(1),
      std::bind(&SpresenseImuSerialNode::pollSerial, this));
  }

private:
  /* CRC‑16/CCITT (poly 0x1021, init 0xFFFF, no reflections) */
  static uint16_t crc16(const uint8_t *data, size_t len)
  {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
      crc ^= static_cast<uint16_t>(data[i]) << 8;
      for (int j = 0; j < 8; ++j) {
        crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
      }
    }
    return crc;
  }

  /* ---------- serial polling ---------- */
  void pollSerial()
  {
    const size_t available = ser_.available();
    if (available == 0) return;

    std::vector<uint8_t> tmp;
    ser_.read(tmp, available);
    buf_.insert(buf_.end(), tmp.begin(), tmp.end());

    /* search & parse frames */
    while (buf_.size() >= FRAME_LEN) {
      /* find sync word 0xAA55 */
      auto it = std::search(buf_.begin(), buf_.end(), SYNC.begin(), SYNC.end());
      if (it == buf_.end()) {
        buf_.clear();
        break;
      }
      size_t pos = std::distance(buf_.begin(), it);
      if (buf_.size() - pos < FRAME_LEN) {
        /* not enough bytes yet */
        if (pos) buf_.erase(buf_.begin(), buf_.begin() + pos);
        break;
      }

      const uint8_t *frame = &buf_[pos];
      uint16_t crc_calc = crc16(frame + 2, PAYLOAD_LEN);
      uint16_t crc_recv = static_cast<uint16_t>(frame[42]) | (static_cast<uint16_t>(frame[43]) << 8);

      if (crc_calc == crc_recv) {
        publishImu(frame + 2);
        buf_.erase(buf_.begin(), buf_.begin() + pos + FRAME_LEN);
      } else {
        /* CRC mismatch → resync by skipping first byte after sync start */
        buf_.erase(buf_.begin() + pos);
      }
    }
  }

  /* ---------- publish Imu message ---------- */
  void publishImu(const uint8_t *payload)
  {
    float f[10];
    std::memcpy(f, payload, PAYLOAD_LEN); // little‑endian to host float

    sensor_msgs::msg::Imu msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;

    msg.orientation.w = f[0];
    msg.orientation.x = f[1];
    msg.orientation.y = f[2];
    msg.orientation.z = f[3];

    msg.angular_velocity.x = f[4];
    msg.angular_velocity.y = f[5];
    msg.angular_velocity.z = f[6];

    msg.linear_acceleration.x = f[7];
    msg.linear_acceleration.y = f[8];
    msg.linear_acceleration.z = f[9];

    imu_pub_->publish(msg);
  }

  /* ---------- constants & members ---------- */
  static constexpr size_t PAYLOAD_LEN = 40;
  static constexpr size_t FRAME_LEN   = 44;
  const std::array<uint8_t, 2> SYNC {0xAA, 0x55};

  serial::Serial ser_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<uint8_t> buf_;
  std::string frame_id_;
};

/* ---------- main ---------- */
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpresenseImuSerialNode>());
  rclcpp::shutdown();
  return 0;
}

