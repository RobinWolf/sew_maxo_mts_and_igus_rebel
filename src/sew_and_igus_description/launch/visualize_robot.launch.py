from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    description_package = "sew_and_igus_description"

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='""',
            description="Prefix for the links and joints in the robot cell",
        )
    )        

    # add optional all other urdf arguments here --> not required for visualization purposes, but take care about them in the real bringup

    tf_prefix = LaunchConfiguration("tf_prefix")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", "sew_and_igus_model.urdf.xacro"]),
            " ",
            "tf_prefix:=",
            tf_prefix,
        ]
    )

    #convert robot_description_content to string, thet it can be passed as yaml to the robot_state_publisher Node --> unsafe without !
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)} 
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    #run joint_state_publisher node, because we don't need to bringup the robot model (without no transforms !) --> only for visualizing porposes     
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    rviz_config_file = PathJoinSubstitution([FindPackageShare(description_package), "rviz", "config.rviz"]) # define path to rviz-config file

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file]
    )
        


    nodes_to_start = [
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)