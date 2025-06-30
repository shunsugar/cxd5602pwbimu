#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <serial/serial.h>

class ImuReceiverNode : public rclcpp::Node
{
public:
  ImuReceiverNode()
  : Node("imu_receiver_node"), serial_("/dev/sensors/Spresense_IMU", 115200, serial::Timeout::simpleTimeout(100))
  {
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_raw", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1), std::bind(&ImuReceiverNode::readSerial, this));
  }

private:
  serial::Serial serial_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<uint8_t> buffer_;

  void readSerial()
  {
    size_t available = serial_.available();
    if (available > 0)
    {
      std::vector<uint8_t> temp;
      serial_.read(temp, available);
      buffer_.insert(buffer_.end(), temp.begin(), temp.end());

      while (buffer_.size() >= 22)
      {
        // 同期（ヘッダ 0x55）
        auto it = std::find(buffer_.begin(), buffer_.end(), 0x55);
        if (it == buffer_.end()) {
          buffer_.clear();
          return;
        }

        size_t pos = std::distance(buffer_.begin(), it);
        if (buffer_.size() < pos + 22) {
          // データが足りない
          break;
        }

        std::vector<uint8_t> packet(buffer_.begin() + pos, buffer_.begin() + pos + 22);

        // チェックサム確認
        uint8_t checksum = 0;
        for (int i = 1; i <= 20; i++) {
          checksum += packet[i];
        }

        if (checksum != packet[21]) {
          // チェックサム不一致
          buffer_.erase(buffer_.begin(), buffer_.begin() + pos + 1);
          continue;
        }

        // デコード
        int16_t data[10];
        for (int i = 0; i < 10; i++) {
          data[i] = (int16_t)(packet[2*i + 2] << 8 | packet[2*i + 1]);
        }

        // 公開
        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = this->now();
        imu_msg.header.frame_id = "imu_link";

        const float ACC_SCALE = 9.80665f / 8192.0f;  // ±4g → 8192LSB/g
        const float GYR_SCALE = (M_PI / 180.0f) / 65.536f; // ±1000dps → 65.536 LSB/deg/s

        imu_msg.linear_acceleration.x = data[0] * ACC_SCALE;
        imu_msg.linear_acceleration.y = data[1] * ACC_SCALE;
        imu_msg.linear_acceleration.z = data[2] * ACC_SCALE;

        imu_msg.angular_velocity.x = data[3] * GYR_SCALE;
        imu_msg.angular_velocity.y = data[4] * GYR_SCALE;
        imu_msg.angular_velocity.z = data[5] * GYR_SCALE;

        imu_msg.orientation.w = data[6] / 10000.0f;
        imu_msg.orientation.x = data[7] / 10000.0f;
        imu_msg.orientation.y = data[8] / 10000.0f;
        imu_msg.orientation.z = data[9] / 10000.0f;

        imu_pub_->publish(imu_msg);

        // 処理済みデータ削除
        buffer_.erase(buffer_.begin(), buffer_.begin() + pos + 22);
      }
    }
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuReceiverNode>());
  rclcpp::shutdown();
  return 0;
}

