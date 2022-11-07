#include "can_msgs/msg/frame.hpp"
#include <boost/asio.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executor.hpp"
#include "std_msgs/msg/int32.hpp"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <memory>
#include <cstdio>
#include <string.h>
// #include "log.h"
#define CAN_FRAME_MAX_LEN 8 

using std::placeholders::_1;
class hw_can_bridge : public rclcpp::Node

{
  public:
    hw_can_bridge();
    ~hw_can_bridge();
    
    int MainLoop();
    void test();

  protected:
    void command_callback(const can_msgs::msg::Frame & msg);
    int InitCanInterface(const char *ifname);
    void state_callback(const can_msgs::msg::Frame & msg);
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr state_subscription_;
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr command_publisher_;
    uint8_t can_data_[CAN_FRAME_MAX_LEN] = { 0xa2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    uint32_t can_id_;
};