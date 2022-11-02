#define CAN_FRAME_MAX_LEN 8 
#include <connect_robot.hpp>
#include <can_interface.hpp>

using std::placeholders::_1;
class CanInterface : public rclcpp::Node

{
  public:
    CanInterface();
    ~CanInterface();
    
    int MainLoop();

  protected:
    void command_callback(const can_msgs::msg::Frame & msg);
    int InitCanInterface(const char *ifname);
    int TransmitCanFrame(const int &sock, const uint32_t &id, const uint8_t *data, const size_t data_len);
    int ReceiveCanFrame(const int sock);
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr subscription_;
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publisher_;
    uint8_t can_data_[CAN_FRAME_MAX_LEN] = { 0xa2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    uint32_t can_id_;
    int sock_;

};

CanInterface::CanInterface()
:Node("CanConnector"), can_id_(0x001)
{
  // subscription_ = this->create_subscription<can_msgs::msg::Frame>(
  //   "can_command", 10, std::bind(&CanInterface::command_callback, this, _1));
  // publisher_ = this->create_publisher<can_msgs::msg::Frame>("can_feedback", 10);


}

CanInterface::~CanInterface()
{
  
}

void CanInterface::command_callback(const can_msgs::msg::Frame & msg)
{
  uint32_t can_id_ = msg.id;
  // can_data_[0] = (msg.data[0]);
  // can_data_[1] = (msg.data[1]);
  // can_data_[2] = (msg.data[2]);
  // can_data_[3] = (msg.data[3]);
  can_data_[4] = (msg.data[4]) & 0xFF;
  can_data_[5] = (msg.data[5]>> 8) & 0xFF;
  can_data_[6] = (msg.data[6] >> 16) & 0xFF;
  can_data_[7] = (msg.data[7] >> 24) & 0xFF;
  
  std::cout << "callback" <<std::endl;
}

int main(int argc, char *argv[])
{
  // rclcpp::init(argc, argv);
  // auto CAN = std::make_shared<CanInterface>();
  // while(rclcpp::ok()){
  //   CAN->MainLoop();
  // }
  
  rclcpp::shutdown();
  return 0;
}