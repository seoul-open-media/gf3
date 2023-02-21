#include "can_msgs/msg/frame.hpp"
#include <boost/asio.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executor.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <memory>
#include <cstdio>
#include <string.h>
#include <iostream>
#include <map>
#include <utility>
// #include "log.h"
#define CAN_FRAME_MAX_LEN 8 

using std::placeholders::_1;
using namespace std::chrono_literals;
namespace gf3_hardware
{
class CanBridge : public rclcpp::Node

{
  public:
    CanBridge();
    ~CanBridge();
    
    int MainLoop();
    void test();
    std::array<uint8_t, 8UL> pub_command(const can_msgs::msg::Frame & msg);
    void state_callback(const can_msgs::msg::Frame & msg);

  protected:
    
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr state_subscription_;
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr command_publisher_;
    uint8_t can_data_[CAN_FRAME_MAX_LEN] = { 0xa2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    uint32_t can_id_;
    std::map<uint8_t, std::array<uint8_t, 8UL>> state_;
    void do_nothing(const can_msgs::msg::Frame & msg);
    
};
}