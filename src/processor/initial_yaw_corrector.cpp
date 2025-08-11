#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class ImuYawCorrectorNode : public rclcpp::Node
{
public:
  ImuYawCorrectorNode() : Node("imu_initial_yaw_corrector")
  {
    declare_parameter("input_topic", "/imu");
    declare_parameter("output_topic", "/imu/fixed");

    get_parameter("input_topic", input_topic_);
    get_parameter("output_topic", output_topic_);

    sub_ = create_subscription<sensor_msgs::msg::Imu>(
      input_topic_, 10,
      std::bind(&ImuYawCorrectorNode::callback, this, std::placeholders::_1));

    pub_ = create_publisher<sensor_msgs::msg::Imu>(output_topic_, 10);

    RCLCPP_INFO(get_logger(), "IMU Yaw Corrector Node started");
  }

private:
  void callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    tf2::Quaternion q;
    tf2::fromMsg(msg->orientation, q);

    // 初期Yaw記録
    if (!yaw_initialized_) {
      tf2::Matrix3x3(q).getRPY(roll_, pitch_, initial_yaw_);
      yaw_initialized_ = true;

      RCLCPP_INFO(get_logger(), "Initial Yaw captured: %.4f rad", initial_yaw_);
    }

    // 現在のRPY
    double r, p, y;
    tf2::Matrix3x3(q).getRPY(r, p, y);

    // 初期Yawを打ち消す補正（roll/pitchはそのまま）
    double corrected_yaw = y - initial_yaw_;
    tf2::Quaternion corrected_q;
    corrected_q.setRPY(r, p, corrected_yaw);
    corrected_q.normalize();

    // 補正したメッセージ作成
    sensor_msgs::msg::Imu corrected_msg = *msg;
    corrected_msg.orientation = tf2::toMsg(corrected_q);

    pub_->publish(corrected_msg);
  }

  std::string input_topic_;
  std::string output_topic_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;

  bool yaw_initialized_ = false;
  double initial_yaw_ = 0.0;
  double roll_ = 0.0, pitch_ = 0.0;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuYawCorrectorNode>());
  rclcpp::shutdown();
  return 0;
}

