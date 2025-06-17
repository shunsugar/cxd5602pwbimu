#include "spresense_imu_node.hpp"

SpresenseImuNode::SpresenseImuNode() : rclcpp::Node("spresense_imu_node")
{
  this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
  this->declare_parameter<std::string>("frame_id", "imu_link");
  this->declare_parameter<std::string>("imu_topic", "/imu");
  this->declare_parameter<float>("time_out", 0.5);
  this->declare_parameter<int>("baudrate", 115200);
  this->get_parameter("port", port_);
  this->get_parameter("frame_id", frame_id_);
  this->get_parameter("imu_topic", imu_topic_);
  this->get_parameter("time_out", time_out_);
  this->get_parameter("baudrate", baudrate_);
  
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 10);
  timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&SpresenseImuNode::processData, this));
  
  openSerial();
}

SpresenseImuNode::~SpresenseImuNode()
{
  if (serial_.isOpen()) {
    serial_.close();
  }
}

void SpresenseImuNode::openSerial()
{
  while (rclcpp::ok()) {
    try {
      // Attempt to open the serial port
      serial_.setPort(port_);
      serial_.setBaudrate(baudrate_);
      serial::Timeout to = serial::Timeout::simpleTimeout(10);
      serial_port_.setTimeout(to);
      serial_.open();
      RCLCPP_INFO(this->get_logger(), "Serial port opened successfully");
      break;
    }
    catch (serial::IOException& e) {
      RCLCPP_WARN(this->get_logger(), "Serial port opening failure... Retrying");
      rclcpp::sleep_for(5s);
    }
  }
}

void SpresenseImuNode::publishMsg





class ImuNode : public rclcpp::Node
{
public:
  SpresenseImuNode()
  : Node("spresense_imu_node"),
    serial_(io_, boost::asio::serial_port_base::baud_rate(921600))
  {
    serial_.open("/dev/ttyUSB0");
    imu_pub_ = create_publisher<Imu>("/imu", 10);
    timer_ = create_wall_timer(
      1ms, std::bind(&SpresenseImuNode::pollSerial, this));
  }

private:
  void pollSerial()
  {
    constexpr size_t FRAME_LEN = 42;             // 0xAA55 + 40Byte + CRC
    static uint8_t buf[FRAME_LEN];
    while (serial_.available() >= FRAME_LEN) {
      serial_.read_some(boost::asio::buffer(buf, FRAME_LEN));
      if (buf[0] != 0xAA || buf[1] != 0x55) continue;  // シンク確認
      /* CRC チェック省略 */

      /* --- デコード --- */
      float* f = reinterpret_cast<float*>(buf + 2);
      Imu msg;
      msg.header.stamp = now();
      msg.header.frame_id = "imu_link";

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

      /* 最低でも対角成分だけでも共分散を入れておくと rviz で色が付く */
      msg.orientation_covariance[0]      = 0.02 * 0.02;
      msg.angular_velocity_covariance[0] = (M_PI/180.0 * 0.1)*(M_PI/180.0*0.1);
      msg.linear_acceleration_covariance[0] = 0.1 * 0.1;

      imu_pub_->publish(msg);
    }
  }

  rclcpp::Publisher<Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  boost::asio::io_service io_;
  boost::asio::serial_port serial_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpresenseImuNode>());
  rclcpp::shutdown();
  return 0;
}

