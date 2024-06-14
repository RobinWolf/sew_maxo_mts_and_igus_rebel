from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
#define the packages for driver and description
    description_package = "sew_agv_description"
    driver_package = "sew_agv_drivers"

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
            "gernerate_ros2_control_tag",
            default_value="false",
            description="Generate the ros2_control tag in the urdf file, if false the tag has to be added manually to the urdf file.",
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
            "robot_ip",
            default_value="10.172.64.1",
            description="The IP-Adress with which the agv hardware joins the common network",
        )
    )
    declared_arguments.append(
    DeclareLaunchArgument('use_sim_time',
            default_value='true',
            description='Set to "true" if you want to use the gazebo clock, set to "fasle" if you use real hardware.'
        )
    )


    tf_prefix = LaunchConfiguration("tf_prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    robot_ip = LaunchConfiguration("robot_ip")
    gernerate_ros2_control_tag = LaunchConfiguration("gernerate_ros2_control_tag")
    ros2_control_with_gazebo = LaunchConfiguration("ros2_control_with_gazebo")
    use_sim_time = LaunchConfiguration('use_sim_time')   



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
            "robot_ip:=",
            robot_ip,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
             " ",
            "gernerate_ros2_control_tag:=",
            gernerate_ros2_control_tag,
            " ",
            "ros2_control_with_gazebo:=",
            ros2_control_with_gazebo,
         ]
    )

    #load the description and the controllers from desired package
    agv_description = {"robot_description": ParameterValue(agv_description_content, value_type=str)} 


    #load the controller manager yaml
    agv_controllers = PathJoinSubstitution([FindPackageShare(driver_package), "config", "agv_controller.yaml"])


    #define the nodes to launch 
    control_node = Node(    #the controller manager is only needed with real hardware, gazebo simulation has its own controller manager !
        condition = UnlessCondition(use_sim_time),
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[agv_description, agv_controllers, {'use_sim_time': use_sim_time}],
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
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)