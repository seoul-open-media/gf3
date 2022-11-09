#include "gf3_hardware/CanBridge.hpp"
namespace gf3_hardware
{
CanBridge::CanBridge()
:Node("HwCanBridge"), can_id_(0x001)
{
  // char *argv[0];
  // rclcpp::init(0, argv);
  
  state_subscription_ = this->create_subscription<can_msgs::msg::Frame>(
    "can_feedback", 1, std::bind(&CanBridge::do_nothing, this, _1));
  command_publisher_ = this->create_publisher<can_msgs::msg::Frame>("can_command", 10);
  
}

CanBridge::~CanBridge()
{
  
}

void CanBridge::do_nothing(const can_msgs::msg::Frame & msg){
}

std::array<uint8_t, 8UL> CanBridge::pub_command(const can_msgs::msg::Frame & command)
{
  command_publisher_->publish(command);  

  rclcpp::TimerBase::SharedPtr one_off_timer;
  rclcpp::WaitSet wait_set({{{state_subscription_}}}, {}, {one_off_timer});

  bool received = false;
  can_msgs::msg::Frame msg;
  rclcpp::MessageInfo msg_info;

  while (!received) {
    const auto wait_result = wait_set.wait(0.1s);
    if (wait_result.kind() == rclcpp::WaitResultKind::Ready) {
      if (wait_result.get_wait_set().get_rcl_wait_set().timers[0U]) {
        one_off_timer->execute_callback();
      } else {
        if (wait_result.get_wait_set().get_rcl_wait_set().subscriptions[0U]) {
          if (state_subscription_->take(msg, msg_info)) {
            received = true;
            // RCLCPP_INFO(this->get_logger(), "msg data: '%x'", msg.id);
          }
        }
      }
    } else if (wait_result.kind() == rclcpp::WaitResultKind::Timeout) {
      if (rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Wait-set failed with timeout");
      }
    }
  }
  // RCLCPP_INFO(this->get_logger(), "Got all messages!");
  // state_.insert_or_assign(msg.id,msg.data);
  return msg.data;
 
}

}

