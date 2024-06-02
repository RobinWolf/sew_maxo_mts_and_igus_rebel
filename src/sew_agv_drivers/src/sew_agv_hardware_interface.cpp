#include "sew_agv_drivers/sew_agv_hardware_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <memory>
#include "pluginlib/class_list_macros.hpp"

namespace sew_agv_drivers
{
hardware_interface::return_type AgvHardwareInterface::configure(const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != hardware_interface::return_type::OK) {
    return hardware_interface::return_type::ERROR;
  }

  // Get parameters
  std::string agv_host = info.hardware_parameters["agv_host"];
  int agv_port = std::stoi(info.hardware_parameters["agv_port"]);
  std::string local_ip = info.hardware_parameters["local_ip"];
  int local_port = std::stoi(info.hardware_parameters["local_port"]);

  // Initialize endpoint
  endpoint_ = std::make_shared<AgvEndpoint>(agv_host, agv_port, local_ip, local_port);

  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> AgvHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &speed_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, "direction_x", &direction_x_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, "direction_y", &direction_y_));
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> AgvHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &speed_));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[0].name, "direction_x", &direction_x_));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[0].name, "direction_y", &direction_y_));
  return command_interfaces;
}

hardware_interface::return_type AgvHardwareInterface::start()
{
  endpoint_->start();
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AgvHardwareInterface::stop()
{
  endpoint_->stop();
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AgvHardwareInterface::read()
{
  // Read data from AGV
  // Implement according to your needs

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AgvHardwareInterface::write()
{
  // Send commands to AGV
  ManualJogTxMsg msg;
  msg.set_speed(speed_);
  msg.set_direction(direction_x_, direction_y_);
  endpoint_->set_msg(msg);

  return hardware_interface::return_type::OK;
}
}  // namespace sew_agv_drivers


PLUGINLIB_EXPORT_CLASS(sew_agv_drivers::AgvHardwareInterface, hardware_interface::SystemInterface)
