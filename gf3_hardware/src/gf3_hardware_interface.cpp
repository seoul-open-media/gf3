#include "gf3_hardware/gf3_hardware_interface.hpp"


#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

// #include "hw_can_bridge.hpp"

namespace gf3_hardware
{
 
  CallbackReturn Gf3HardwareInterface::on_init(
    const hardware_interface::HardwareInfo & info)
  {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    CAN_ = std::make_shared<CanBridge>();

    // START: This part here is for exemplary purposes - Please do not copy to your production code
    // hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
    // hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
    // hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);
    // END: This part here is for exemplary purposes - Please do not copy to your production code
    
    hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    prev_hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    for (const hardware_interface::ComponentInfo & joint : info_.joints)
    {
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("Gf3HardwareInterface"),
          "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
          joint.command_interfaces.size());
        return CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("Gf3HardwareInterface"),
          "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("Gf3HardwareInterface"),
          "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
        return CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("Gf3HardwareInterface"),
          "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return CallbackReturn::ERROR;
      }
    }

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn Gf3HardwareInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
  {
    for (uint i = 0; i < hw_states_.size(); i++)
    {
      hw_states_[i] = 0;
      prev_hw_states_[i] = 0;
      hw_commands_[i] = 0;
    }

    RCLCPP_INFO(rclcpp::get_logger("Gf3HardwareInterface"), "Successfully configured!");

    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface>
  Gf3HardwareInterface::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface>
  Gf3HardwareInterface::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
    }

    return command_interfaces;
  }

  CallbackReturn Gf3HardwareInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // command and state should be equal when starting
    for (uint i = 0; i < hw_states_.size(); i++)
    {
      hw_commands_[i] = hw_states_[i];
    }

    RCLCPP_INFO(rclcpp::get_logger("Gf3HardwareInterface"), "Successfully activated!");

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn Gf3HardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // START: This part here is for exemplary purposes - Please do not copy to your production code
    // RCLCPP_INFO(
    //   rclcpp::get_logger("Gf3HardwareInterface"), "Deactivating ...please wait...");

    // for (int i = 0; i < hw_stop_sec_; i++)
    // {
    //   rclcpp::sleep_for(std::chrono::seconds(1));
    //   RCLCPP_INFO(
    //     rclcpp::get_logger("Gf3HardwareInterface"), "%.1f seconds left...",
    //     hw_stop_sec_ - i);
    // }

    // RCLCPP_INFO(rclcpp::get_logger("Gf3HardwareInterface"), "Successfully deactivated!");
    // END: This part here is for exemplary purposes - Please do not copy to your production code

    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type Gf3HardwareInterface::read()
  {
    // for (uint8_t idx = 0 ;idx < info_.joints.size(); idx++){
    for (uint8_t idx = 0 ;idx < 3; idx++){
      auto reply = CAN_->pub_command(ARM_->getPosition(motor_ID[idx]));
      // hw_states_[0] =((reply[6] & 0xFF ) + ((reply[7] << 8) & 0xFF00)) / 100 * M_PI / 180 / 36;
      if (idx == 0){ // V1 com protocol
        int64_t raw_position = ((reply[1] & 0xFF ) +
                                ((reply[2] << 8) & 0xFF'00) +
                                ((reply[3] << 16) & 0xFF'00'00) +
                                ((reply[4] << 24) & 0xFF'00'00'00) +
                                ((reply[5] << 32) & 0xFF'00'00'00'00) +
                                ((reply[6] << 40) & 0xFF'00'00'00'00'00) +
                                ((reply[7] << 48) & 0xFF'00'00'00'00'00'00));

        if (reply[6] == 0xFF && reply[7] == 0xFF){
          raw_position = (raw_position) - 0x00'00'00'FF'FF'FF'FF - 0x00'00'00'00'00'00'01;
        }
        hw_states_[idx] =(raw_position / 100 * M_PI / 180 / 36);
      }
      else // V3 com protocol
      {
        int64_t raw_position = ((reply[4] & 0xFF) +
                                ((reply[5] << 8) & 0xFF'00) +
                                ((reply[6] << 16) & 0xFF'00'00) +
                                ((reply[7] << 24) & 0xFF'00'00'00));

        if (reply[7] == 0xFF){
          raw_position = (raw_position) - 0x00'00'00'FF'FF'FF'FF - 0x00'00'00'00'00'00'01;
        }
        hw_states_[idx] =-(raw_position / 100 * M_PI / 180);
      }
    }

    
    
    // START: This part here is for exemplary purposes - Please do not copy to your production code
    // RCLCPP_INFO(rclcpp::get_logger("Gf3HardwareInterface"), "Reading...");

    // for (uint i = 0; i < hw_states_.size(); i++)
    // {
    //   // Simulate RRBot's movement
    //   hw_states_[i] = hw_states_[i] + (hw_commands_[i] - hw_states_[i]) / hw_slowdown_;
    //   RCLCPP_INFO(
    //     rclcpp::get_logger("Gf3HardwareInterface"), "Got state %.5f for joint %d!",
    //     hw_states_[i], i);
    // }
    // RCLCPP_INFO(rclcpp::get_logger("Gf3HardwareInterface"), "Joints successfully read!");
    // END: This part here is for exemplary purposes - Please do not copy to your production code

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type Gf3HardwareInterface::write()
  {
    // START: This part here is for exemplary purposes - Please do not copy to your production code
    // RCLCPP_INFO(rclcpp::get_logger("Gf3HardwareInterface"), "Writing...");

    // for (uint i = 0; i < hw_commands_.size(); i++)
    // {
    //   // Simulate sending commands to the hardware
    //   RCLCPP_INFO(
    //     rclcpp::get_logger("Gf3HardwareInterface"), "Got command %.5f for joint %d!",
    //     hw_commands_[i], i);
    // }
    // RCLCPP_INFO(
    //   rclcpp::get_logger("Gf3HardwareInterface"), "Joints successfully written!");
    // END: This part here is for exemplary purposes - Please do not copy to your production code
    int ratio = 0;
    for (uint8_t idx = 0 ;idx < 3; idx++){
      if (idx == 0) ratio = 36; else ratio = 1;
      int32_t raw_command = hw_commands_[idx] * 100 / M_PI * 180 * ratio;
      std::array<uint8_t, 8UL> command;
      // command[0] = gf3_hardware::Commands::Myactuator::SET_POS_COMMAND;
      command[0] = 0xA4;
      command[1] = 0x00;
      command[2] = 0xB0;
      if (idx == 0) command[3] = 0x06;
      else{
        command[3] = 0x00;
        raw_command = -raw_command;
      }
      // command[3] = 0x00;
      command[4] = raw_command & 0x00;
      command[5] = (raw_command >> 8) & 0xFF;
      command[6] = (raw_command >> 16) & 0xFF;
      command[7] = (raw_command >> 24) & 0xFF;
      // std::cout << "writting" << std::endl;
      CAN_->pub_command(ARM_->setPosition(motor_ID[idx], command));
    }
    return hardware_interface::return_type::OK;
  }

}  // namespace

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  gf3_hardware::Gf3HardwareInterface, hardware_interface::SystemInterface)