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

    hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    prev_hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    // set motor-zero position offset from yaml

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
    // for (uint i = 0; i < hw_states_.size(); i++)
    // {
    //   hw_states_[i] = 0;
    //   prev_hw_states_[i] = 0;
    //   hw_commands_[i] = 0;
    //   if(i==1) hw_states_ [i] = -0.25;
    // }
       for (uint i = 0; i < hw_states_.size(); i++)
    {
      hw_commands_[i] = hw_states_[i];
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

    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type Gf3HardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // for (uint8_t idx = 0 ;idx < info_.joints.size(); idx++){
    for (uint8_t idx = 0; idx < 4; idx++){
      auto reply = CAN_->pub_command(ARM_->getPosition(motor_ID[idx]));
      // hw_states_[0] =((reply[6] & 0xFF ) + ((reply[7] << 8) & 0xFF00)) / 100 * M_PI / 180 / 36;

      // if (idx == 0){ // V1 com protocol
      //   int64_t raw_position = ((reply[1] & 0xFF ) +
      //                           ((reply[2] << 8) & 0xFF'00) +
      //                           ((reply[3] << 16) & 0xFF'00'00) +
      //                           ((reply[4] << 24) & 0xFF'00'00'00) +
      //                           ((reply[5] << 32) & 0xFF'00'00'00'00) +
      //                           ((reply[6] << 40) & 0xFF'00'00'00'00'00) +
      //                           ((reply[7] << 48) & 0xFF'00'00'00'00'00'00));

      //   if (reply[6] == 0xFF && reply[7] == 0xFF){
      //     raw_position = (raw_position) - 0x00'00'00'FF'FF'FF'FF - 0x00'00'00'00'00'00'01;
      //   }
      //   hw_states_[idx] =(raw_position / 100 * M_PI / 180 / 36);
      // }

      // V3 com protocol
      double raw_position = ((reply[4] & 0xFF) +
                              ((reply[5] << 8) & 0xFF'00) +
                              ((reply[6] << 16) & 0xFF'00'00) +
                              ((reply[7] << 24) & 0xFF'00'00'00));
      

      if (reply[7] == 0xFF){
        raw_position = (raw_position) - 0x00'00'00'FF'FF'FF'FF - 0x00'00'00'00'00'00'01;
      }
      raw_position = raw_position/100;
      if (idx == 1){
        raw_position -= 22.5;
      }

      if (idx == 2){
        raw_position += 4.4;
      }

      hw_states_[idx] =-(raw_position * M_PI / 180);
  
    }
    hw_states_[4] = 0;

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type Gf3HardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    int ratio = 1;
    for (uint8_t idx = 0 ;idx < 4; idx++){
      // if (idx == 0) ratio = 36; else ratio = 1;
      int32_t raw_command = hw_commands_[idx]  / M_PI * 180 * ratio;
      std::array<uint8_t, 8UL> command;
      // command[0] = gf3_hardware::Commands::Myactuator::SET_POS_COMMAND;  // default position command
      command[0] = 0xA5; // position command with velocity limit
      command[1] = 0x12;
      command[2] = 0x00;
      // command[2] = 0x00;

      // **** for V1 protocol ****
      // if (idx == 0) command[3] = 0x04;
      // else{
      //   command[3] = 0x00;
      //   raw_command = -raw_command;
      // }

      if (idx == 1){
        raw_command -= 22.5;
      }

      if (idx == 2){
        raw_command -= 4.4;
      }

      raw_command = -raw_command * 100;
      command[3] = 0x00;
      command[4] = raw_command & 0xFF;
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