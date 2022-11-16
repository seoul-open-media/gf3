#include "can_msgs/msg/frame.hpp"
#include <boost/asio.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executor.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <memory>
#include <cstdio>
#include <string.h>
#include <iostream>
#include <algorithm>
#include <map>
#include <utility>
// #include "log.h"
#define CAN_FRAME_MAX_LEN 8 

using std::placeholders::_1;
using namespace std::chrono_literals;
namespace gf3_hardware
{
class MecanumControl : public rclcpp::Node

{
  public:
    MecanumControl();
    ~MecanumControl();
    std::array<uint8_t, 8UL> pub_command(const can_msgs::msg::Frame & msg);
    void state_callback(const can_msgs::msg::Frame & msg);
    int MainLoop();
    void test();

  protected:
    
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_state_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_command_publisher_;
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr odom_publisher_;
    // rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_command_publisher_;

    uint8_t can_data_[CAN_FRAME_MAX_LEN] = { 0xcf, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x07 };
    uint32_t can_id_;
    std::map<uint8_t, std::array<uint8_t, 8UL>> state_;
    void cmd_vel_callback(const geometry_msgs::msg::Twist &msg);
    void can_reply_callback(const can_msgs::msg::Frame &msg);
    
};
}