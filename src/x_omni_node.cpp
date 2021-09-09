#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"

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

    publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("odom_vel", 10);
    subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "cmd_vel", 10, std::bind(&XOmniNode::topic_callback, this, _1));

    timer_ = this->create_wall_timer(
      10ms, std::bind(&XOmniNode::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto msg = geometry_msgs::msg::Vector3();
    _serial->update();
    if(_serial->read() == 0){
      msg.x = this->odom.data.x;
      msg.y = this->odom.data.y;
      msg.z = this->odom.data.th;
      publisher_->publish(msg);
    }
    _serial->write(1);
  }

  void topic_callback(const geometry_msgs::msg::Vector3::ConstSharedPtr msg)
  {
    this->cmd.data.x = msg->x;
    this->cmd.data.y = msg->y;
    this->cmd.data.th = msg->z;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscription_;

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
