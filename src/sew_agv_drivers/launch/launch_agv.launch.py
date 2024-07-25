from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
#define the packages for driver and description
    description_package = "sew_agv_description"
    driver_package = "sew_agv_drivers"
    navigation_package = "sew_agv_navigation"

#declare launch arguments (can be passed in the command line while launching)
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="sew_",
            description="Prefix for the links and joints of the agv",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="start the agv with fake (mock) hardware or real controller",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "generate_ros2_control_tag",
            default_value="true",
            description="Generate the ros2_control tag in the urdf file, if false the tag has to be added manually to the urdf file.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="Start RViz2 automatically with this launch file, should be deactivated when launching moveit from this base image.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value='false',
            description='Set to "true" if you want to use the gazebo clock, set to "fasle" if you use real hardware.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ros2_control_with_gazebo",
            default_value='false',
            description="add the robot description to gazebo ros2 control for the diff_drive, no gazebo internal plugin!",
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
            "agv_ip",
            default_value="192.168.10.4",  #agv_link PLC IP
            description="The IP address of the AGV",
        )
    )  
    declared_arguments.append(
        DeclareLaunchArgument(
            "agv_port",
            default_value="11000",  #agv_link PLC IP
            description="The port number of the AGV.",
        )
    )  
    declared_arguments.append(
        DeclareLaunchArgument(
            "local_ip",
            default_value="192.168.10.222",
            description="The local IP address to bind the UDP socket",
        )
    )  
    declared_arguments.append(
        DeclareLaunchArgument(
            "local_port",
            default_value="11001",
            description="The local port number to bind the UDP socket.",
        )
    )  
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_wheel_name",
            default_value="sew_wheel_left_joint",
            description="Name of the left wheel.",
        )
    )  
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_wheel_name",
            default_value="sew_wheel_right_joint",
            description="Name of the right wheel.",
        )
    )  
    declared_arguments.append(
        DeclareLaunchArgument(
            "wheel_separation",
            default_value="1.0",
            description="Wheel seperation of the agv.",
        )
    )  
    declared_arguments.append(
        DeclareLaunchArgument(
            "wheel_radius",
            default_value="0.1",
            description="Wheel radius of the agv.",
        )
    )  
    declared_arguments.append(
        DeclareLaunchArgument(
            "enable_joystick",
            default_value="true",
            description="set to true if you want to use a joystick (XBox controller od keyboard) to move the robot.",
        )
    ) 
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_controller',
            default_value='true',
            description='Set to "true" if you want to use a xBox One Controller to control the AGV movement, set to "false" if you want to use "wasd" on the keyboard instead.',
        )
    )

    tf_prefix = LaunchConfiguration("tf_prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    rviz = LaunchConfiguration("rviz")
    generate_ros2_control_tag = LaunchConfiguration("generate_ros2_control_tag")
    use_sim_time = LaunchConfiguration('use_sim_time')
    ros2_control_with_gazebo = LaunchConfiguration("ros2_control_with_gazebo")
    standalone_gazebo = LaunchConfiguration("standalone_gazebo")
    agv_ip = LaunchConfiguration("agv_ip")
    agv_port = LaunchConfiguration("agv_port")
    local_ip = LaunchConfiguration("local_ip")
    local_port = LaunchConfiguration("local_port")
    left_wheel_name = LaunchConfiguration("left_wheel_name")
    right_wheel_name = LaunchConfiguration("right_wheel_name")
    wheel_separation = LaunchConfiguration("wheel_separation")
    wheel_radius = LaunchConfiguration("wheel_radius")
    enable_joystick = LaunchConfiguration("enable_joystick")
    use_controller = LaunchConfiguration("use_controller")


#define the robot description content
    agv_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", "sew_agv_model.urdf.xacro"]), 
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
             " ",
            "generate_ros2_control_tag:=",
            generate_ros2_control_tag,
            " ",
            "ros2_control_with_gazebo:=",
            ros2_control_with_gazebo,
            " ",
            "standalone_gazebo:=",
            standalone_gazebo,
            " ",
            "agv_ip:=",
            agv_ip,
            " ",
            "agv_port:=",
            agv_port,
            " ",
            "local_ip:=",
            local_ip,
            " ",
            "local_port:=",
            local_port,
            " ",
            "left_wheel_name:=",
            left_wheel_name,
            " ",
            "right_wheel_name:=",
            right_wheel_name,
            " ",
            "wheel_separation:=",
            wheel_separation,
            " ",
            "wheel_radius:=",
            wheel_radius,
         ]
    )

    #load the description and the controllers from desired package
    agv_description = {"robot_description": ParameterValue(agv_description_content, value_type=str)} 

    #load the rviz config file with visualization settings
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "rviz_config.rviz"]
    )

    #load the controller manager yaml
    agv_controllers = PathJoinSubstitution([FindPackageShare(driver_package), "config", "agv_controller.yaml"])


#define the nodes to launch 
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[agv_description, agv_controllers, {'use_sim_time': use_sim_time}],  #,{"tf_prefix": tf_prefix, "tf_prefix_arm": tf_prefix_arm}
        output="both",
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[agv_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(rviz)
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
    )

    # Include des Joystick Launch-Files
    joystick_launch = IncludeLaunchDescription(
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


    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )
    nodes_to_start = [
        control_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        robot_state_pub_node,
        joystick_launch,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)