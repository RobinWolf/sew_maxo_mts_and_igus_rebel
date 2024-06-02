from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue
from launch.conditions import IfCondition


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
    # default_value="127.0.0.1"
    # default_value="192.168.10.5",
    declared_arguments.append(
        DeclareLaunchArgument(
            "agv_ip",
            default_value="10.172.64.1",
            description="The IP-Adress with which the agv hardware joins the common network",
        )
    )  

    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="Start RViz2 automatically with this launch file, should be deactivated when launching moveit from this base image.",
        )
    )


    tf_prefix = LaunchConfiguration("tf_prefix")

    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    agv_ip = LaunchConfiguration("agv_ip")

    rviz = LaunchConfiguration("rviz")
    gernerate_ros2_control_tag = LaunchConfiguration("gernerate_ros2_control_tag")



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
            "agv_ip:=",
            agv_ip,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
             " ",
            "gernerate_ros2_control_tag:=",
            gernerate_ros2_control_tag,
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
        parameters=[agv_description, agv_controllers],  #,{"tf_prefix": tf_prefix, "tf_prefix_arm": tf_prefix_arm}
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
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)