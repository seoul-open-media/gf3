#include "gf3_hardware/hw_can_bridge.hpp"

hw_can_bridge::hw_can_bridge()
:Node("HwCanBridge"), can_id_(0x001)
{
  // char *argv[0];
  // rclcpp::init(0, argv);
 state_subscription_ = this->create_subscription<can_msgs::msg::Frame>(
    "can_feedback", 10, std::bind(&hw_can_bridge::state_callback, this, _1));
  command_publisher_ = this->create_publisher<can_msgs::msg::Frame>("can_command", 10);
}

hw_can_bridge::~hw_can_bridge()
{
  
}

void hw_can_bridge::command_callback(const can_msgs::msg::Frame & msg)
{
  uint32_t can_id_ = msg.id;
  can_data_[0] = (msg.data[0]);
  can_data_[1] = (msg.data[1]);
  can_data_[2] = (msg.data[2]);
  can_data_[3] = (msg.data[3]);
  can_data_[4] = (msg.data[4]) & 0xFF;
  can_data_[5] = (msg.data[5]>> 8) & 0xFF;
  can_data_[6] = (msg.data[6] >> 16) & 0xFF;
  can_data_[7] = (msg.data[7] >> 24) & 0xFF;
  
  std::cout << "callback" <<std::endl;
}

int hw_can_bridge::InitCanInterface(const char *ifname)
{
  return 0;
}

void hw_can_bridge::state_callback(const can_msgs::msg::Frame & msg)
{
  uint32_t can_id_ = msg.id;
  can_data_[0] = (msg.data[0]);
  can_data_[1] = (msg.data[1]);
  can_data_[2] = (msg.data[2]);
  can_data_[3] = (msg.data[3]);
  can_data_[4] = (msg.data[4]) & 0xFF;
  can_data_[5] = (msg.data[5]>> 8) & 0xFF;
  can_data_[6] = (msg.data[6] >> 16) & 0xFF;
  can_data_[7] = (msg.data[7] >> 24) & 0xFF;
  
  std::cout << "callback" <<std::endl;
}
int hw_can_bridge::MainLoop()
{
  rclcpp::spin_some(shared_from_this());
  return 0;
}

void hw_can_bridge::test()
{
  std::cout << "test" << std::endl;

  struct can_frame frame;
// a200000000001000

  auto command_msg = can_msgs::msg::Frame();
  command_msg.id = 0x141;
  command_msg.data[0] = 0xA2;
  command_msg.data[1] = 0x00;
  command_msg.data[2] = 0x00;
  command_msg.data[3] = 0x00;
  command_msg.data[4] = 0x00;
  command_msg.data[5] = 0x00;
  command_msg.data[6] = 0x10;
  command_msg.data[7] = 0x00;
  // command_msg.id = command_msg.can_id ;
  // for(uint8_t i = 0; i<=7; i++){
  //   command_msg.data[i] = frame.data[i];
  // }

  command_publisher_->publish(command_msg);
}

// int main(int argc, char *argv[])
// {
//   rclcpp::init(argc, argv);
//   auto CAN = std::make_shared<hw_can_bridge>();
//   while(rclcpp::ok()){
//     CAN->MainLoop();
//   }
  
//   rclcpp::shutdown();
//   return 0;
// }