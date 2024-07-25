#include "sew_agv_drivers/sew_agv_hardware_interface.hpp"



namespace sew_agv_drivers {
    
  SewAgvHardwareInterface::~SewAgvHardwareInterface() {
    //virtual destructor enables proper cleanup in polymorphic class hierarchies by ensuring the correct destructor is invoked for objects of derived classes when deleted through a base class pointer
    on_deactivate(rclcpp_lifecycle::State());
  }

  hardware_interface::CallbackReturn SewAgvHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
  {
    RCLCPP_INFO(rclcpp::get_logger("SewAgvHardwareInterface"), "Initializing ...");
    if (
      hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // read parameters from the ros2_control.xacro file
    cfg_.agv_ip = info_.hardware_parameters["agv_ip"];
    cfg_.agv_port = std::stoi(info_.hardware_parameters["agv_port"]);
    cfg_.local_ip = info_.hardware_parameters["local_ip"];
    cfg_.local_port = std::stoi(info_.hardware_parameters["local_port"]);
    
    wheels_.left_wheel_.name = info_.hardware_parameters["left_wheel_name"];
    wheels_.right_wheel_.name = info_.hardware_parameters["right_wheel_name"];
    wheels_.wheel_separation_ = std::stod(info_.hardware_parameters["wheel_separation"]);
    wheels_.wheel_radius_ = std::stod(info_.hardware_parameters["wheel_radius"]);

    // Log information about parameters
    RCLCPP_INFO(rclcpp::get_logger("SewAgvHardwareInterface"), "AGV IP: %s", cfg_.agv_ip.c_str());
    RCLCPP_INFO(rclcpp::get_logger("SewAgvHardwareInterface"), "AGV Port: %d", cfg_.agv_port);
    RCLCPP_INFO(rclcpp::get_logger("SewAgvHardwareInterface"), "Local IP: %s", cfg_.local_ip.c_str());
    RCLCPP_INFO(rclcpp::get_logger("SewAgvHardwareInterface"), "Local Port: %d", cfg_.local_port);
    RCLCPP_INFO(rclcpp::get_logger("SewAgvHardwareInterface"), "Left Wheel Name: %s", wheels_.left_wheel_.name.c_str());
    RCLCPP_INFO(rclcpp::get_logger("SewAgvHardwareInterface"), "Right Wheel Name: %s", wheels_.right_wheel_.name.c_str());
    RCLCPP_INFO(rclcpp::get_logger("SewAgvHardwareInterface"), "Wheel Separation: %f", wheels_.wheel_separation_);
    RCLCPP_INFO(rclcpp::get_logger("SewAgvHardwareInterface"), "Wheel Radius: %f", wheels_.wheel_radius_);
    
    for (const hardware_interface::ComponentInfo & joint : info_.joints)
    {
      // DiffBotSystem has exactly two states and one command interface on each joint
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("SewAgvHardwareInterface"),
          "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
          joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("SewAgvHardwareInterface"),
          "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("SewAgvHardwareInterface"),
          "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("SewAgvHardwareInterface"),
          "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("SewAgvHardwareInterface"),
          "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    RCLCPP_INFO(rclcpp::get_logger("SewAgvHardwareInterface"), "Finished initializing successfully.");
    return hardware_interface::CallbackReturn::SUCCESS;
  }



  // export_state_interfaces and export_command_interfaces to tell ros2_control what the hardware interface has accessable
  std::vector<hardware_interface::StateInterface> SewAgvHardwareInterface::export_state_interfaces()
  {
    RCLCPP_INFO(rclcpp::get_logger("SewAgvHardwareInterface"), "Start exporting state interfaces ...");

    std::vector<hardware_interface::StateInterface> state_interfaces;

    // link the state interface of the controller with the variables of wheels_
    state_interfaces.emplace_back(hardware_interface::StateInterface(wheels_.left_wheel_.name, hardware_interface::HW_IF_VELOCITY, &wheels_.left_wheel_.vel));
    state_interfaces.emplace_back(hardware_interface::StateInterface(wheels_.left_wheel_.name, hardware_interface::HW_IF_POSITION, &wheels_.left_wheel_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(wheels_.right_wheel_.name, hardware_interface::HW_IF_VELOCITY, &wheels_.right_wheel_.vel));
    state_interfaces.emplace_back(hardware_interface::StateInterface(wheels_.right_wheel_.name, hardware_interface::HW_IF_POSITION, &wheels_.right_wheel_.pos));

    RCLCPP_INFO(rclcpp::get_logger("SewAgvHardwareInterface"), "Export of state interfaces to controller manager finished sucessfully");

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> SewAgvHardwareInterface::export_command_interfaces()
  {
    RCLCPP_INFO(rclcpp::get_logger("SewAgvHardwareInterface"), "Start exporting command interfaces ...");

    std::vector<hardware_interface::CommandInterface> command_interfaces;
    
    // link the command interface of the controller with the variables of wheels_
    command_interfaces.emplace_back(hardware_interface::CommandInterface(wheels_.left_wheel_.name, hardware_interface::HW_IF_VELOCITY, &wheels_.left_wheel_.cmd));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(wheels_.right_wheel_.name, hardware_interface::HW_IF_VELOCITY, &wheels_.right_wheel_.cmd));


    RCLCPP_INFO(rclcpp::get_logger("SewAgvHardwareInterface"), "Export of command interfaces to controller manager finished sucessfully");

    return command_interfaces;
  }



  hardware_interface::CallbackReturn SewAgvHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("SewAgvHardwareInterface"), "Activating ...");

    // Connect to AGV
    bool connected = agv_endpoint_.connect(cfg_.agv_ip, cfg_.agv_port, cfg_.local_ip, cfg_.local_port);

    // check if connection was successful
    if (connected) {
        RCLCPP_INFO(rclcpp::get_logger("SewAgvHardwareInterface"), "Successfully connected to AGV.");
        return hardware_interface::CallbackReturn::SUCCESS;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("SewAgvHardwareInterface"), "Failed to connect to AGV.");
        return hardware_interface::CallbackReturn::ERROR;
    }
  }



  hardware_interface::CallbackReturn SewAgvHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("SewAgvHardwareInterface"), "Deactivating ...");

    // Disconnect from AGV
    agv_endpoint_.disconnect();

    RCLCPP_INFO(rclcpp::get_logger("SewAgvHardwareInterface"), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }



  hardware_interface::return_type SewAgvHardwareInterface::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // Function to read status from AGV
    // No position feedback from AGV, only status --> no need to read from AGV, status is read once in on_activate
    return hardware_interface::return_type::OK;

    // Reading from AGV is not needed in every cycle, but this code shows youw, how to read the status
    // if(agv_endpoint_.readAgvRxBuffer())
    // {
    //     RCLCPP_INFO(rclcpp::get_logger("SewAgvHardwareInterface"), "Successfully read from AGV.");
    //     return hardware_interface::return_type::OK;
    // }
    // else
    // {
    //     RCLCPP_WARN(rclcpp::get_logger("SewAgvHardwareInterface"), "Failed to read from AGV.");
    //     return hardware_interface::return_type::OK;
    // }
  };



  hardware_interface::return_type SewAgvHardwareInterface::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    float speed = 0.0;
    float x = 0.0;
    float y = 0.0;
    wheels_.getVelocityAndDirection(speed, x, y);     // Calculate the speed, x and y values from the wheel velocities that are set in the diffdrive controller
    
    
    // Send command to agv
    if(agv_endpoint_.writeAgvTxBuffer(speed, x, y))
    {
      //RCLCPP_INFO(rclcpp::get_logger("SewAgvHardwareInterface"), "Successfully sent control to AGV. Speed: %f, X: %f, Y: %f", speed, x, y);
      return hardware_interface::return_type::OK;
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("SewAgvHardwareInterface"), "Failed to send control to AGV.");
      return hardware_interface::return_type::ERROR;
    }
  };
}  // namespace sew_agv_drivers


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  sew_agv_drivers::SewAgvHardwareInterface,
  hardware_interface::SystemInterface
)
