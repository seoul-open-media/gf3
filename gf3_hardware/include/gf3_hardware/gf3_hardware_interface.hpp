
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "can_msgs/msg/frame.hpp"
#include "gf3_hardware/hw_can_bridge.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace gf3_hardware
{
  
  class Gf3HardwareInterface : public hardware_interface::SystemInterface
  {
  public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Gf3HardwareInterface);

    
    CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

    
    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

    
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    
    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    
    hardware_interface::return_type read() override;

    
    hardware_interface::return_type write() override;

  private:
  // Parameters for the RRBot simulation
    double hw_start_sec_;
    double hw_stop_sec_;
    double hw_slowdown_;

    // void state_callback(const can_msgs::msg::Frame & msg);
    // int send_command(const can_msgs::msg::Frame & msg);

    // // Set publisher and subscriber for command and feedback(state)
    // // rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr state_subscription_;
    // // rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr command_publisher_;
    // uint8_t can_data_[8] = { 0xa2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    // uint32_t can_id_;
    std::shared_ptr<hw_can_bridge> CAN_;


    std::vector<double> hw_commands_;
    std::vector<double> hw_states_;
    
  };
}
