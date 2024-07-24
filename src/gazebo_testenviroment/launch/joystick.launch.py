from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    navigation_package = "sew_agv_navigation"

    #declare launch arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Set to "true" if you want to use the joystick with gazebo, set to "fasle" if you use real hardware.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "generate_ros2_control_tag",
            default_value='false',
            description="launch hardware drivers for real hardware, mock hardware or gazebo (recommendet to use)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "standalone_gazebo",
            default_value='true',
            description="add the robot description to gazebo with a simpler approach, using a diff_drive and lidar plugin (NOT recommendet to use, only for testing purposes without ros2 control)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_controller',
            default_value='true',
            description='Set to "true" if you want to use a xBox One Controller to control the AGV movement, set to "false" if you want to use "wasd" on the keyboard instead.',
        )
    )
    
    
    use_sim_time = LaunchConfiguration('use_sim_time')   
    use_controller = LaunchConfiguration('use_controller')
    standalone_gazebo = LaunchConfiguration("standalone_gazebo")
    generate_ros2_control_tag = LaunchConfiguration('generate_ros2_control_tag')
    
    #define nodes to launch
    joy_params = os.path.join(get_package_share_directory(navigation_package),'config', 'joystick', 'joystick.yaml')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{'dev': '/dev/input/js0'}],  # Specify the device file for your joystick
        condition=IfCondition(use_controller)
    )

    teleop_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_joy_node',
        output='screen',
        parameters=[joy_params,{use_sim_time}],  # enable teleop of the robot by pressing X on the xBox controller
        remappings=[('/cmd_vel', '/cmd_vel_joy')],
        condition=IfCondition(use_controller)
    )

    keyboard_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        remappings=[('/cmd_vel', '/cmd_vel_joy')],
        prefix='xterm -e',  # open a new terminal window where you can control the robot with the keyboard
        condition=UnlessCondition(use_controller)
    )

    twistmux_params = os.path.join(get_package_share_directory(navigation_package),'config', 'joystick', 'twistmux.yaml')

    twistmux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twistmux_params,{use_sim_time}], 
        remappings=[('/cmd_vel_out', '/cmd_vel')], # until ros2_control is available just shortcut the twistmux node --> controller is not prioritized!
        condition=IfCondition(standalone_gazebo)
   
    )

    twistmux_node_control = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twistmux_params,{use_sim_time}],  
        remappings=[('/cmd_vel_out', '/diffbot_base_controller/cmd_vel_unstamped')],
        condition=IfCondition(generate_ros2_control_tag)
    )


    nodes_to_start = [teleop_joy_node, joy_node, twistmux_node, twistmux_node_control, keyboard_node]

    return LaunchDescription(declared_arguments + nodes_to_start)