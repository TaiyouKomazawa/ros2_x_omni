#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <SerialBridge.hpp>
#include <LinuxHardwareSerial.hpp>

#include "Uint8Data.h"
#include "SbVector3.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

SerialDev *dev = new LinuxHardwareSerial("/dev/ttyUSB0", B115200);
SerialBridge serial(dev);

class XOmniNode : public rclcpp::Node
{
public:
  XOmniNode(SerialBridge *serial)
      : Node("x_omni_node"), _serial(serial)
  {
    _serial->add_frame(0, &this->odom);
    _serial->add_frame(1, &this->cmd);

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("odom_vel", 10);
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&XOmniNode::topic_callback, this, _1));

    timer_ = this->create_wall_timer(
      50ms, std::bind(&XOmniNode::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto msg = geometry_msgs::msg::Twist();
    _serial->update();

    if(_serial->read() == 0){
      RCLCPP_INFO(this->get_logger(), "read odom.");
      msg.linear.x = this->odom.data.x;
      msg.linear.y = this->odom.data.y;
      msg.angular.z = this->odom.data.th;
      publisher_->publish(msg);
    }

    _serial->write(1);
  }

  void topic_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
  {
    this->cmd.data.x = msg->linear.x;
    this->cmd.data.y = msg->linear.y;
    this->cmd.data.th = msg->angular.z;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;

  SerialBridge *_serial;
  SbVector3 odom, cmd;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<XOmniNode>(&serial));
  rclcpp::shutdown();
  return 0;
}
