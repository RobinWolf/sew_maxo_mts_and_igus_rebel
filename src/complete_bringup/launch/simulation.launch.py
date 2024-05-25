from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition


def generate_launch_description():
    sim_package = "gazebo_testenviroment"
    igus_moveit_package = "sew_and_igus_moveit_config"
    navigation_package = "sew_agv_navigation"

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='sew_',
            description="Prefix for the links and joints from tha SEW AGV",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "standalone_gazebo",
            default_value='true',
            description="add the robot description to gazebo with a simpler approach, using a diff_drive and lidar plugin",
        )
    )
    declared_arguments.append(
    DeclareLaunchArgument('world',
            default_value='src/gazebo_testenviroment/worlds/static_world_2404.world',
            description='Specify the world which should be loaded in Gazebo'
        )
    )
    declared_arguments.append(
    DeclareLaunchArgument('use_sim_time',
            default_value='true',
            description='Set to "true" if you want to use the gazebo clock, set to "fasle" if you use real hardware.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "enable_joystick",
            default_value='true',
            description="set to true if you want to use a joystick (XBox controller) to move the robot",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "hardware_protocol",
            default_value='gazebo',
            description="select the hardware protocol which should be used by ros2 control resource manager",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='igus_',
            description="set to true if you want to use a joystick (XBox controller) to move the robot",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value='true',
            description="set to true if rviz gui should be launched by default",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_mapping",
            default_value='false',
            description="set to true if you want to launch the slam toolbox for creating a new map",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_navigation",
            default_value='false',
            description="set to true if you want to launch navigation",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_moveit",
            default_value='true',
            description="set to true if you want to launch moveit for the igus arm",
        )
    )


    #init launch arguments, transfer to variables
    world = LaunchConfiguration('world')        
    tf_prefix = LaunchConfiguration("tf_prefix")
    standalone_gazebo = LaunchConfiguration("standalone_gazebo")
    use_sim_time = LaunchConfiguration('use_sim_time')   
    enable_joystick = LaunchConfiguration('enable_joystick') 
    prefix = LaunchConfiguration('prefix') 
    hardware_protocol = LaunchConfiguration('hardware_protocol') 
    launch_rviz = LaunchConfiguration('launch_rviz') 
    launch_mapping = LaunchConfiguration('launch_mapping')
    launch_navigation = LaunchConfiguration('launch_navigation')
    launch_moveit = LaunchConfiguration('launch_moveit')



    #lauch gazebo if launch argument is set to true (without ros2 control)
    load_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare(sim_package), 'launch']), "/robot.launch.py"]),
            condition=IfCondition(standalone_gazebo),
            launch_arguments={
                "tf_prefix": tf_prefix,
                "world": world,
                "standalone_gazebo": standalone_gazebo,
                'prefix': prefix,
                'hardware_protocol': hardware_protocol,
            }.items(),
    )

    #launch the joystick
    load_joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare(navigation_package), 'launch']), "/joystick.launch.py"]),
            condition=IfCondition(enable_joystick),
            launch_arguments={
                "use_sim_time": use_sim_time,
            }.items(),
    )

    #launch igus driver with gazebo ros2 control hardware interface and moveit if launch argument is set to true
    load_igus = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare(igus_moveit_package), 'launch']), "/controller_and_moveit.launch.py"]),
            condition=IfCondition(launch_moveit),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "launch_rviz" : launch_rviz
            }.items(),
    )

    #launch navigation if launch launch argument is set to true
    load_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare(navigation_package), 'launch']), "/navigation.launch.py"]),
            condition=IfCondition(launch_navigation),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "launch_rviz" : launch_rviz
            }.items(),
    )
    # delay_load_navigation = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=load_navigation,
    #         on_exit=[load_gazebo],
    #     )
    # )

    #launch mapping if launch argument is set to true
    load_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare(navigation_package), 'launch']), "/mapping.launch.py"]),
            condition=IfCondition(launch_mapping),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "launch_rviz" : launch_rviz
            }.items(),
    )


    nodes_to_start = [
        load_gazebo,
        load_joystick,
        load_igus,
        load_mapping,
        load_navigation,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
