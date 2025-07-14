#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "MadgwickAHRS.h"  // 自前で持ってくる

class ImuFilterNode : public rclcpp::Node
{
public:
  ImuFilterNode() : Node("imu_filter_node")
  {
    sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/Spresense", 10, std::bind(&ImuFilterNode::imu_callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/madgwick", 10);
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // 例: Madgwickの入力形式に変換
    float gx = msg->angular_velocity.x;
    float gy = msg->angular_velocity.y;
    float gz = msg->angular_velocity.z;
    float ax = msg->linear_acceleration.x;
    float ay = msg->linear_acceleration.y;
    float az = msg->linear_acceleration.z;

    filter.updateIMU(gx, gy, gz, ax, ay, az);

    // 姿勢（クォータニオン）を取得して出力
    sensor_msgs::msg::Imu filtered_msg = *msg;
    filtered_msg.orientation.w = filter.q0;
    filtered_msg.orientation.x = filter.q1;
    filtered_msg.orientation.y = filter.q2;
    filtered_msg.orientation.z = filter.q3;

    pub_->publish(filtered_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;

  Madgwick filter;
};

