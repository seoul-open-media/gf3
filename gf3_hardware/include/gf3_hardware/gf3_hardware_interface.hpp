
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
#include "gf3_hardware/CanBridge.hpp"
#include "gf3_hardware/myactuator.hpp"
#include "gf3_hardware/MoteusAPI.h"

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

    
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  private:
    double hw_start_sec_;
    double hw_stop_sec_;
    double hw_slowdown_;
    std::vector<double> hw_commands_;
    std::vector<double> hw_states_, prev_hw_states_;
    
    std::shared_ptr<MoteusAPI> MOTEUS_;
    // std::shared_ptr<CanBridge> CAN_;
    // std::shared_ptr<Myactuator> ARM_;

    const uint32_t motor_ID[4] ={0x141U, 0x142U, 0x143U, 0x144U};
  };
}
