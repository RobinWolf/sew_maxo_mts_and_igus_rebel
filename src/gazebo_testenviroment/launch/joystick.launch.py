from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    sim_package = "gazebo_testenviroment"

    #declare launch arguments
    declared_arguments = []
    declared_arguments.append(
    DeclareLaunchArgument('use_sim_time', default_value='true',
                            description='Set to "true" if you want to use the joystick with gazebo, set to "fasle" if you use real hardware.'))
    
    use_sim_time = LaunchConfiguration('use_sim_time')   
    
    #define nodes to launch
    joy_params = os.path.join(get_package_share_directory(sim_package),'config','joystick.yaml')
    teleop_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_joy_node',
        output='screen',
        parameters=[joy_params,{use_sim_time}]  # enable teleop of the robot by pressing X on the xBox controller
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{'dev': '/dev/input/js0'}]  # Specify the device file for your joystick
    )

    nodes_to_start = [teleop_joy_node, joy_node]
    return LaunchDescription(declared_arguments + nodes_to_start)