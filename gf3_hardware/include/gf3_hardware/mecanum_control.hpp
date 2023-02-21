#include "can_msgs/msg/frame.hpp"
#include <boost/asio.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/clock.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
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
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    

  protected:
    
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_state_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_command_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    // rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_command_publisher_;

    uint8_t can_data_[CAN_FRAME_MAX_LEN] = { 0xcf, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x07 };
    uint32_t can_id_;
    std::map<uint8_t, std::array<uint8_t, 8UL>> state_;
    void cmd_vel_callback(const geometry_msgs::msg::Twist &msg);
    void can_reply_callback(const can_msgs::msg::Frame &msg);
    double received_FL_w_, received_FR_w_, received_RL_w_,received_RR_w_;
    bool is_FL_received_, is_FR_received_, is_RL_received_, is_RR_received_;
    nav_msgs::msg::Odometry odom_;
    can_msgs::msg::Frame m1_command_, m2_command_;
    double x_, y_, th_;
    double prev_time_;
};
}