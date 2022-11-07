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

    CAN_ = std::make_shared<hw_can_bridge>();
    CAN_->test();

    // START: This part here is for exemplary purposes - Please do not copy to your production code
    hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
    hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
    hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);
    // END: This part here is for exemplary purposes - Please do not copy to your production code
    hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
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
    // START: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(
      rclcpp::get_logger("Gf3HardwareInterface"), "Configuring ...please wait...");

    for (int i = 0; i < hw_start_sec_; i++)
    {
      rclcpp::sleep_for(std::chrono::seconds(1));
      RCLCPP_INFO(
        rclcpp::get_logger("Gf3HardwareInterface"), "%.1f seconds left...",
        hw_start_sec_ - i);
    }
    // END: This part here is for exemplary purposes - Please do not copy to your production code

    // reset values always when configuring hardware
    for (uint i = 0; i < hw_states_.size(); i++)
    {
      hw_states_[i] = 0;
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
    // START: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(
      rclcpp::get_logger("Gf3HardwareInterface"), "Activating ...please wait...");

    for (int i = 0; i < hw_start_sec_; i++)
    {
      rclcpp::sleep_for(std::chrono::seconds(1));
      RCLCPP_INFO(
        rclcpp::get_logger("Gf3HardwareInterface"), "%.1f seconds left...",
        hw_start_sec_ - i);
    }
    // END: This part here is for exemplary purposes - Please do not copy to your production code

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
    RCLCPP_INFO(
      rclcpp::get_logger("Gf3HardwareInterface"), "Deactivating ...please wait...");

    for (int i = 0; i < hw_stop_sec_; i++)
    {
      rclcpp::sleep_for(std::chrono::seconds(1));
      RCLCPP_INFO(
        rclcpp::get_logger("Gf3HardwareInterface"), "%.1f seconds left...",
        hw_stop_sec_ - i);
    }

    RCLCPP_INFO(rclcpp::get_logger("Gf3HardwareInterface"), "Successfully deactivated!");
    // END: This part here is for exemplary purposes - Please do not copy to your production code

    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type Gf3HardwareInterface::read()
  {
    // START: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(rclcpp::get_logger("Gf3HardwareInterface"), "Reading...");

    for (uint i = 0; i < hw_states_.size(); i++)
    {
      // Simulate RRBot's movement
      hw_states_[i] = hw_states_[i] + (hw_commands_[i] - hw_states_[i]) / hw_slowdown_;
      RCLCPP_INFO(
        rclcpp::get_logger("Gf3HardwareInterface"), "Got state %.5f for joint %d!",
        hw_states_[i], i);
    }
    RCLCPP_INFO(rclcpp::get_logger("Gf3HardwareInterface"), "Joints successfully read!");
    // END: This part here is for exemplary purposes - Please do not copy to your production code

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type Gf3HardwareInterface::write()
  {
    // START: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(rclcpp::get_logger("Gf3HardwareInterface"), "Writing...");

    for (uint i = 0; i < hw_commands_.size(); i++)
    {
      // Simulate sending commands to the hardware
      RCLCPP_INFO(
        rclcpp::get_logger("Gf3HardwareInterface"), "Got command %.5f for joint %d!",
        hw_commands_[i], i);
    }
    RCLCPP_INFO(
      rclcpp::get_logger("Gf3HardwareInterface"), "Joints successfully written!");
    // END: This part here is for exemplary purposes - Please do not copy to your production code

    return hardware_interface::return_type::OK;
  }

}  // namespace

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  gf3_hardware::Gf3HardwareInterface, hardware_interface::SystemInterface)