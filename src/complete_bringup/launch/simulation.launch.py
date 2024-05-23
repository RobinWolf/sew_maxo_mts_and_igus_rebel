from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition


def generate_launch_description():
    description_package = "sew_and_igus_description"
    sim_package = "gazebo_testenviroment"
    navigation_package = "sew_agv_navigation"
    igus_bringup_package = "irc_ros_bringup"
    moveit_package = "irc_ros_moveit_config"

    declared_arguments = []
    #arguments to configure sew agv
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='""',
            description="Prefix for the links and joints in the robot cell",
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
            "generate_ros2_control_tag",
            default_value='false',
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
        DeclareLaunchArgument(
            "enable_joystick",
            default_value='true',
            description="set to true if you want to use a joystick (XBox controller) to move the robot",
        )
    )
    #arguments to confugure igus rebel
    namespace_arg = DeclareLaunchArgument("namespace", default_value="")
    prefix_arg = DeclareLaunchArgument("prefix", default_value="")
    controller_manager_name_arg = DeclareLaunchArgument(
        "controller_manager_name",
        default_value=[LaunchConfiguration("namespace"), "/controller_manager"],
    )

    default_rviz_file = PathJoinSubstitution(
        [FindPackageShare("irc_ros_description"), "rviz", "rebel.rviz"]
    )
    default_urdf_filename_arg = DeclareLaunchArgument(
        "default_urdf_filename",
        default_value=[LaunchConfiguration("robot_name"), ".urdf.xacro"],
        description="Name of the robot description file",
    )
    default_urdf_file = PathJoinSubstitution(
        [
            FindPackageShare("irc_ros_description"),
            "urdf",
            LaunchConfiguration("default_urdf_filename"),
        ]
    )
    default_robot_controller_filename_arg = DeclareLaunchArgument(
        "default_robot_controller_filename",
        default_value=["controller_", LaunchConfiguration("robot_name"), ".yaml"],
        description="Name of the robot controller configuration file",
    )
    default_robot_controller_file = PathJoinSubstitution(
        [
            FindPackageShare("irc_ros_bringup"),
            "config",
            LaunchConfiguration("default_robot_controller_filename"),
        ]
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        choices=["0", "1", "false", "true", "False", "True"],
        description="Whether to start rviz with the launch file",
    )
    rviz_file_arg = DeclareLaunchArgument(
        "rviz_file",
        default_value=default_rviz_file,
        description="The path to the rviz configuration file",
    )
    robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        default_value="igus_rebel_6dof",
        choices=["igus_rebel_6dof", "igus_rebel_4dof"],
        description="Which igus ReBeL type to use",
    )
    robot_urdf_arg = DeclareLaunchArgument(
        "robot_urdf",
        default_value=default_urdf_file,
        description="Path of the robot description file",
    )
    robot_controller_config_arg = DeclareLaunchArgument(
        "robot_controller_config",
        default_value=default_robot_controller_file,
        description="Path of the robot's description file",
    )
    rebel_version_arg = DeclareLaunchArgument(
        "rebel_version",
        default_value="01",
        choices=["pre", "00", "01"],
        description="Which version of the igus ReBeL to use",
    )
    gripper_arg = DeclareLaunchArgument(
        "gripper",
        default_value="none",
        choices=["none", "schmalz_ecbpmi", "ext_dio_gripper"],
        description="Which gripper to attach to the flange",
    )
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="The namespace to use for all nodes started by this launch file",
    )

    hardware_protocol_arg = DeclareLaunchArgument(
        "hardware_protocol",
        default_value="cprcanv2",
        choices=["mock_hardware", "gazebo", "cprcanv2", "cri"],
        description="Which hardware protocol or mock hardware should be used",
    )

    #init launch arguments, transfer to variables
    #sew arguments
    world = LaunchConfiguration('world')        
    tf_prefix = LaunchConfiguration("tf_prefix")
    standalone_gazebo = LaunchConfiguration("standalone_gazebo")
    use_sim_time = LaunchConfiguration('use_sim_time')   
    generate_ros2_control_tag = LaunchConfiguration('generate_ros2_control_tag')   
    robot_ip = LaunchConfiguration('robot_ip') 
    enable_joystick = LaunchConfiguration('enable_joystick') 
    #igus arguments
    namespace = LaunchConfiguration("namespace")
    prefix = LaunchConfiguration("prefix")
    controller_manager_name = LaunchConfiguration("controller_manager_name")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_file = LaunchConfiguration("rviz_file")
    robot_urdf = LaunchConfiguration("robot_urdf")
    robot_controller_config_file = LaunchConfiguration("robot_controller_config")
    rebel_version = LaunchConfiguration("rebel_version")
    gripper = LaunchConfiguration("gripper")
    hardware_protocol = LaunchConfiguration("hardware_protocol")

    
    #launch sew agv bringup    
    load_agv = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare(navigation_package), 'launch']), "/bringup.launch.py"]),
            launch_arguments={
                "tf_prefix": tf_prefix,
                "world": world,
                "standalone_gazebo": standalone_gazebo,
                'launch_rviz':'false',
                ...
            }.items(),
    )

    #include igus bringup
    load_igus_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare(igus_bringup_package), 'launch']), "/rebel.launch.py"]),
            launch_arguments={

                ...
            }.items(),
    )

    #include igus moveit
    load_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare(moveit_package), 'launch']), "/rebel.launch.py"]),
            launch_arguments={

                ...
            }.items(),
    )




    nodes_to_start = [
        load_agv,
        load_igus_bringup,
        load_moveit
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
