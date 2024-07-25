#ifndef SEW_AGV_HARDWARE_INTERFACE_HPP
#define SEW_AGV_HARDWARE_INTERFACE_HPP

//#include <cstring>
#include <string>
#include "rclcpp/rclcpp.hpp"

//#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
//#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "sew_agv_drivers/agv_endpoint.hpp"
#include "sew_agv_drivers/config.h"
#include "sew_agv_drivers/wheels_to_vel_and_dir.hpp"

using hardware_interface::return_type;

namespace sew_agv_drivers {
  class SewAgvHardwareInterface : public hardware_interface::SystemInterface
  {

  public:
    virtual ~SewAgvHardwareInterface();

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override; //declares a member function named on_init. It is declared as an override of a function in the base class

    // hardware_interface::CallbackReturn on_configure(const   rclcpp_lifecycle::State& previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  private:
    Config cfg_;
    Wheels_to_vel_and_dir wheels_;
    AgvEndpoint agv_endpoint_;

    // rclcpp::Logger logger_;
    // std::chrono::time_point<std::chrono::system_clock> time_;
    
    };
  };  // namespace sew_agv_drivers

  #endif // SEW_AGV_HARDWARE_INTERFACE_HPP
