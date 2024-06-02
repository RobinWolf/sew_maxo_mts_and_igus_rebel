#ifndef SEW_AGV_DRIVERS__AGV_HARDWARE_INTERFACE_HPP_
#define SEW_AGV_DRIVERS__AGV_HARDWARE_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <vector>
#include <string>
#include <memory>

#include "agv_endpoint.hpp"

namespace sew_agv_drivers
{
class AgvHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(AgvHardwareInterface)

  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type start() override;
  hardware_interface::return_type stop() override;
  hardware_interface::return_type read() override;
  hardware_interface::return_type write() override;

private:
  std::shared_ptr<AgvEndpoint> endpoint_;
  double speed_;
  double direction_x_;
  double direction_y_;
};
}  // namespace sew_agv_drivers

#endif  SEW_AGV_DRIVERS__AGV_HARDWARE_INTERFACE_HPP_
