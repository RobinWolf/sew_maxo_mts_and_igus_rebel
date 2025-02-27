from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition


def generate_launch_description():
    sim_package = "gazebo_testenviroment"
    igus_moveit_package = "sew_and_igus_moveit_config"
    navigation_package = "sew_agv_navigation"
    bringup_package = "complete_bringup"
    driver_package = "sew_agv_drivers"
    description_package = "sew_and_igus_description"

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
            default_value='false',
            description="add the robot description to gazebo with a simpler approach, using a diff_drive and lidar plugin",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ros2_control_with_gazebo",
            default_value='true',
            description="add the robot description to gazebo ros2 control for the diff_drive (recommendet to use instead of standalone_gazebo)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "generate_ros2_control_tag",
            default_value='true',
            description="launch the drivers that connect to the real hardware via IP",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value='TODO',
            description="the IP the real robot can be pinged",
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
            'use_controller',
            default_value='true',
            description='Set to "true" if you want to use a xBox One Controller to control the AGV movement, set to "false" if you want to use "wasd" on the keyboard instead.',
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
            "launch_mapping",
            default_value='false',
            description="set to true if you want to launch the slam toolbox for creating a new map",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_navigation",
            default_value='true',
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
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_seperate_rviz",
            default_value='false',
            description="set to true if seperate rviz guis for navigationa and moveit should be launched by default",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_complete_rviz",
            default_value='true',
            description="set to true if one rviz gui for all (moveit and navigation) should be launched by default",
        )
    )


    #init launch arguments, transfer to variables
    world = LaunchConfiguration('world')        
    tf_prefix = LaunchConfiguration("tf_prefix")
    standalone_gazebo = LaunchConfiguration("standalone_gazebo")
    generate_ros2_control_tag = LaunchConfiguration('generate_ros2_control_tag') 
    ros2_control_with_gazebo = LaunchConfiguration("ros2_control_with_gazebo")
    robot_ip = LaunchConfiguration('robot_ip') 
    use_sim_time = LaunchConfiguration('use_sim_time')   
    enable_joystick = LaunchConfiguration('enable_joystick')
    use_controller = LaunchConfiguration('use_controller') 
    prefix = LaunchConfiguration('prefix') 
    launch_complete_rviz = LaunchConfiguration('launch_complete_rviz')
    launch_seperate_rviz = LaunchConfiguration('launch_seperate_rviz') 
    launch_mapping = LaunchConfiguration('launch_mapping')
    launch_navigation = LaunchConfiguration('launch_navigation')
    launch_moveit = LaunchConfiguration('launch_moveit')



    #lauch gazebo without condition (simulation.launch!)
    load_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare(sim_package), 'launch']), "/robot.launch.py"]),
            launch_arguments={
                "tf_prefix": tf_prefix,
                "prefix": prefix,
                "world": world,
                "standalone_gazebo": standalone_gazebo,
                "generate_ros2_control_tag": generate_ros2_control_tag,
                "ros2_control_with_gazebo": ros2_control_with_gazebo,
                'launch_rviz':'false',
            }.items(),
    )

    #launch the joystick
    load_joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare(navigation_package), 'launch']), "/joystick.launch.py"]),
            condition=IfCondition(enable_joystick),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "use_controller": use_controller,
                "standalone_gazebo": standalone_gazebo,
                "generate_ros2_control_tag": generate_ros2_control_tag,
            }.items(),
    )

    #launch sew drivers if launch argument is set to true
    load_sew_drivers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare(driver_package), 'launch']), "/driver.launch.py"]),
            condition=IfCondition(generate_ros2_control_tag),
            launch_arguments={
                "tf_prefix": tf_prefix,
                "robot_ip": robot_ip,
                "generate_ros2_control_tag": generate_ros2_control_tag,
                "ros2_control_with_gazebo": ros2_control_with_gazebo,
                "use_sim_time": use_sim_time,
            }.items(),
    )

    delay_load_sew_drivers = TimerAction(
        period=10.0,  # Delay period in seconds
        actions=[load_sew_drivers]
    )

    #launch igus driver with gazebo ros2 control hardware interface and moveit if launch argument is set to true
    load_igus = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare(igus_moveit_package), 'launch']), "/controller_and_moveit.launch.py"]),
            condition=IfCondition(launch_moveit),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "standalone_gazebo": standalone_gazebo,
                "generate_ros2_control_tag": generate_ros2_control_tag,
                "ros2_control_with_gazebo": ros2_control_with_gazebo,
                "launch_rviz" : launch_seperate_rviz
            }.items(),
    )
    delay_load_igus = TimerAction(
        period=15.0,  # Delay period in seconds
        actions=[load_igus]
    )

    #launch navigation if launch launch argument is set to true
    load_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare(navigation_package), 'launch']), "/navigation.launch.py"]),
            condition=IfCondition(launch_navigation),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "launch_rviz" : launch_seperate_rviz
            }.items(),
    )
    delay_load_navigation = TimerAction(
        period=20.0,  # Delay period in seconds
        actions=[load_navigation]
    )

    #launch mapping if launch argument is set to true
    load_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare(navigation_package), 'launch']), "/mapping.launch.py"]),
            condition=IfCondition(launch_mapping),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "launch_rviz" : launch_seperate_rviz
            }.items(),
    )
    delay_load_mapping = TimerAction(
        period=20.0,
        actions=[load_mapping]
    )

    #launch whole rviz node with stored config and motion planning parameters
    rviz_config_file = PathJoinSubstitution([FindPackageShare(bringup_package), "rviz", "moveit_and_nav2.rviz"]) # define path to rviz-config file
    robot_description_kinematics_file = PathJoinSubstitution(   # needed for the interactive markers in rviz
        [
            FindPackageShare(igus_moveit_package),
            "config",
            "kinematics.yaml",
        ]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            {'use_sim_time': use_sim_time},
            robot_description_kinematics_file,
        ],
        condition=IfCondition(launch_complete_rviz)
    )
    delay_rviz_node= TimerAction(
        period=25.0,
        actions=[rviz_node]
    )


    #use delays to ensure that gazebo server is launched first, if not you will suffer from unpredictable problems!
    #gazebo and joystick -> moveit -> mapping or navigation -> rviz
    nodes_to_start = [
        load_gazebo,
        load_joystick,
        delay_load_sew_drivers,
        delay_load_igus,
        delay_load_mapping,
        delay_load_navigation,
        delay_rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
