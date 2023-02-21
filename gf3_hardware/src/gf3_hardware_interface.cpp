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
      const hardware_interface::HardwareInfo &info)
  {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }
    string dev_name("/dev/ttyACM0");
    int moteus_id = 1;

    MOTEUS_ = std::make_shared<MoteusAPI>(dev_name, moteus_id);
    // MoteusAPI api(dev_name, moteus_id);

    // CAN_ = std::make_shared<CanBridge>();

    hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    prev_hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    // set motor-zero position offset from yaml

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
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
    State curr_state;

    // only read current position
    MOTEUS_->ReadState(curr_state.EN_Position());
    cout << "position: " << curr_state.position << endl;

    // reset the state and only read velocity and torque
    // curr_state.Reset();
    // MOTEUS_->ReadState(curr_state.EN_Velocity().EN_Torque());

    // // read temperature in addition to velocity and torque
    // MOTEUS_->ReadState(curr_state.EN_Temp());

    // print everyting
    // cout << "velocity: " << curr_state.velocity << endl;
    // cout << "torque: " << curr_state.torque << endl;
    // cout << "temperature: " << curr_state.temperature << endl;
    for (uint i = 0; i < hw_states_.size(); i++)
    {
      hw_states_[i] = curr_state.position;
    }
    // RCLCPP_INFO(rclcpp::get_logger("Gf3HardwareInterface"), "read!");

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type Gf3HardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // send one position with speed and torque limits
    double stop_position = 0;
    double velocity = 0.05;
    double max_torque = 1;
    double feedforward_torque = 0;
    MOTEUS_->SendPositionCommand(stop_position, velocity, max_torque,
                                 feedforward_torque);
    return hardware_interface::return_type::OK;
  }

} // namespace

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    gf3_hardware::Gf3HardwareInterface, hardware_interface::SystemInterface)