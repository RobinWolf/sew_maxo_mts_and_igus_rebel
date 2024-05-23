from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    navigation_package = "sew_agv_navigation"

    #declare launch arguments
    declared_arguments = []
    declared_arguments.append(
    DeclareLaunchArgument('use_sim_time', default_value='false',
                            description='Set to "true" if you want to use the joystick with gazebo, set to "fasle" if you use real hardware.'))
    
    use_sim_time = LaunchConfiguration('use_sim_time')   
    
    #define nodes to launch
    joy_params = os.path.join(get_package_share_directory(navigation_package),'config', 'joystick', 'joystick.yaml')
    teleop_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_joy_node',
        output='screen',
        parameters=[joy_params,{use_sim_time}],  # enable teleop of the robot by pressing X on the xBox controller
        remappings=[('/cmd_vel', '/cmd_vel_joy')]
    )

    twistmux_params = os.path.join(get_package_share_directory(navigation_package),'config', 'joystick', 'twistmux.yaml')
    twistmux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twistmux_params,{use_sim_time}],  # just needs to be set up, because topic remapping in gazebo without ros2_control is not possible
        remappings=[('/cmd_vel_out', '/cmd_vel')]   # until ros2_control is available just shortcut the twistmux node --> controller is not prioritized!
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{'dev': '/dev/input/js0'}]  # Specify the device file for your joystick
    )

    nodes_to_start = [teleop_joy_node, joy_node, twistmux_node]
    return LaunchDescription(declared_arguments + nodes_to_start)