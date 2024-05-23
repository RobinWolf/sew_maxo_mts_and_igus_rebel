from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():


    param_file_name = 'mapping.yaml' 
    navigation_package = "sew_agv_navigation"

    default_param_dir = os.path.join(get_package_share_directory(navigation_package), 'config', 'navigation' ,param_file_name)
    

    declared_arguments = []
    declared_arguments.append(
    DeclareLaunchArgument('use_sim_time',
            default_value='true',
            description='Set to "true" if you want to use the gazebo clock, set to "fasle" if you use real hardware.'
        )
    )
    declared_arguments.append(
    DeclareLaunchArgument('param_dir',
            default_value=default_param_dir,
            description='pass the path to your mapping.yaml file where params for slam_toolbox node are defined.'
        )
    )
    declared_arguments.append(
    DeclareLaunchArgument('launch_rviz',
            default_value='true',
            description='set to true if you want to launch the rviz gui to view the mapping process'
        )
    )


    #init launch arguments, transfer to variables
    use_sim_time = LaunchConfiguration('use_sim_time')     
    param_dir = LaunchConfiguration('param_dir')     
    launch_rviz = LaunchConfiguration('launch_rviz')     



    #add nodes
    slam_toolbox_node = Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            output='screen',
            name='slam_toolbox',
            parameters = [param_dir])
    
    rviz_config_file = PathJoinSubstitution([FindPackageShare(navigation_package), "rviz", "mapping.rviz"]) # define path to rviz-config file

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz)
    )




    nodes_to_start = [
        slam_toolbox_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)



#save map in new terminal with: ~/ros2_ws$ ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/sew_agv_navigation/config/navigation/<name>

