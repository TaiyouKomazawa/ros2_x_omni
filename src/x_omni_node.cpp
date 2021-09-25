#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ros2_x_omni_msgs/msg/imu_axis9.hpp"

#include <SerialBridge.hpp>
#include <LinuxHardwareSerial.hpp>

#include "Uint8Data.h"
#include "SbVector3.h"
#include "Axis9Sensor.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

class XOmniNode : public rclcpp::Node
{
public:
  XOmniNode()
      : Node("x_omni_node")
  {
    declare_parameter("serial_port", "/dev/ttyXOmni");
    auto port = this->get_parameter("serial_port").as_string();
    _serial = new SerialBridge(new LinuxHardwareSerial(port.c_str(), B2000000), 1024);

    _serial->add_frame(0, &this->odom);
    _serial->add_frame(1, &this->cmd);
    _serial->add_frame(2, &this->msense_msg);
    _serial->add_frame(3, &this->rst);

    this->dev_reset();

    this->_get_cmd = false;

    odom_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("x_omni/odom_vel/raw", 10);
    imu_pub_ = this->create_publisher<ros2_x_omni_msgs::msg::ImuAxis9>("x_omni/imu/raw", 10);
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&XOmniNode::topic_callback, this, _1));

    poller_timer_ = this->create_wall_timer(
        5ms, std::bind(&XOmniNode::polling_callback, this));
    sender_timer_ = this->create_wall_timer(
        50ms, std::bind(&XOmniNode::sending_callback, this));
  }

  ~XOmniNode()
  {
    delete _serial;
  }

  void dev_reset()
  {
    RCLCPP_INFO(this->get_logger(), "x omni device reset.");
    this->rst.data.c = 1;
    _serial->write(3);
    rclcpp::sleep_for(500ms);
  }

private:
  void polling_callback()
  {
    auto odom_msg = geometry_msgs::msg::Twist();
    auto imu_msg = ros2_x_omni_msgs::msg::ImuAxis9();
    _serial->update();

    if(_serial->read() == 0){
      //RCLCPP_INFO(this->get_logger(), "read odom.");
      odom_msg.linear.x = this->odom.data.x;
      odom_msg.linear.y = this->odom.data.y;
      odom_msg.angular.z = this->odom.data.z;
      odom_pub_->publish(odom_msg);
      imu_msg.header.stamp  = rclcpp::Clock().now();
      imu_msg.header.frame_id = "imu_frame";
      imu_msg.data[0] = this->msense_msg.data.acc.x;
      imu_msg.data[1] = this->msense_msg.data.acc.y;
      imu_msg.data[2] = this->msense_msg.data.acc.z;
      imu_msg.data[3] = this->msense_msg.data.gyro.x;
      imu_msg.data[4] = this->msense_msg.data.gyro.y;
      imu_msg.data[5] = this->msense_msg.data.gyro.z;
      imu_msg.data[6] = this->msense_msg.data.mag.x;
      imu_msg.data[7] = this->msense_msg.data.mag.y;
      imu_msg.data[8] = this->msense_msg.data.mag.z;
      imu_pub_->publish(imu_msg);
    }
  }

  void sending_callback()
  {
    if (this->_get_cmd){
      _serial->write(1);
      this->_get_cmd = false;
    }
  }

  void topic_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
  {
    this->cmd.data.x = msg->linear.x;
    this->cmd.data.y = msg->linear.y;
    this->cmd.data.z = msg->angular.z;
    this->_get_cmd = true;
  }

  rclcpp::TimerBase::SharedPtr poller_timer_, sender_timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr odom_pub_;
  rclcpp::Publisher<ros2_x_omni_msgs::msg::ImuAxis9>::SharedPtr imu_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;

  SerialBridge *_serial;
  SbVector3 odom, cmd;
  Axis9Sensor msense_msg;
  Uint8Data rst;

  bool _get_cmd;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<XOmniNode>();
  rclcpp::spin(node);
  node->dev_reset();
  rclcpp::shutdown();
  return 0;
}
