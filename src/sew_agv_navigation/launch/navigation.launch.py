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

from nav2_common.launch import RewrittenYaml

def generate_launch_description():


    param_file_name = 'navigation_experimental.yaml' 
    map_file_name = '240512_raw.yaml' 
    navigation_package = "sew_agv_navigation"

    default_param_dir = os.path.join(get_package_share_directory(navigation_package), 'config', 'navigation', param_file_name)
    default_map_dir = os.path.join(get_package_share_directory(navigation_package), 'config', 'navigation', 'maps', map_file_name)
    

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='sew_',
            description="Prefix for the links and joints of the agv to avoid name collisions",
        )
    )
    declared_arguments.append(
    DeclareLaunchArgument('use_sim_time',
            default_value='true',
            description='Set to "true" if you want to use the gazebo clock, set to "false" if you use real hardware.'
        )
    )
    declared_arguments.append(
    DeclareLaunchArgument('autostart',
            default_value='true',
            description='Set to "true" if you want to start the nav2 lifecycle automatically in launch.'
        )
    )
    declared_arguments.append(
    DeclareLaunchArgument('param_dir',
            default_value=default_param_dir,
            description='pass the path to your navigation.yaml file where params for all navigation and localization node are defined.'
        )
    )
    declared_arguments.append(
    DeclareLaunchArgument('map_yaml_file',
            default_value=default_map_dir,
            description='pass the path to your <map_name>.yaml file where params for all navigation and localization node are defined.'
        )
    )
    declared_arguments.append(
    DeclareLaunchArgument('launch_rviz',
            default_value='true',
            description='set to true if you want to launch the rviz gui to view the mapping process'
        )
    )

    #init launch arguments, transfer to variables
    tf_prefix = LaunchConfiguration('tf_prefix')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')  
    param_dir = LaunchConfiguration('param_dir')    
    map_yaml_file = LaunchConfiguration('map_yaml_file')  
    launch_rviz = LaunchConfiguration('launch_rviz')     
 

    #modify default navigation.yaml file with parameters passed as lauch arguments (only for map_server to change maps)
    param_substitutions = {
        'use_sim_time' : use_sim_time,
        'yaml_filename': map_yaml_file,
        'tf_prefix' : tf_prefix}        #prefix substitution does not work, please redefine manually in the navigation.yaml file

    configured_param_dir = RewrittenYaml(
        source_file=param_dir,
        param_rewrites=param_substitutions,
        convert_types=True)

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    #########################################################################################################################
    ###                                            nodes for neo_localization                                             ###
    #########################################################################################################################
    #all params for these nodes are defined in the navigation.yaml file. Every section of this file connects to the drsired node name
    map_server = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[configured_param_dir], 
            remappings=remappings)
    
    amcl_localization_node = Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[configured_param_dir],
            remappings=remappings)
    
    lifecycle_nodes_localization = ['map_server', 'amcl']

    lifecycle_manager_localization = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes_localization}])  
    
    #########################################################################################################################
    ###                                               nodes for neo_navigation                                            ###
    #########################################################################################################################
    controller_server = Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                parameters=[configured_param_dir])
    planner_server = Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[configured_param_dir])
    behavior_server = Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[configured_param_dir])
    bt_navigator = Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[configured_param_dir])
    waypoint_follower = Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                parameters=[configured_param_dir])
    
    lifecycle_nodes_navigation = ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator', 'waypoint_follower']
    lifecycle_manager_navigation = Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes_navigation}])


    #########################################################################################################################
    ###                                                           common                                                  ###
    #########################################################################################################################

    rviz_config_file = PathJoinSubstitution([FindPackageShare(navigation_package), "rviz", "navigation.rviz"]) # define path to rviz-config file

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz)
    )


    nodes_to_start = [
        rviz_node,
        lifecycle_manager_localization,
        map_server,
        amcl_localization_node,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        lifecycle_manager_navigation
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
