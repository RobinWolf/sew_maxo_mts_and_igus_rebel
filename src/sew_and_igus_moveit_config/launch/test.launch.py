import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ParameterValue
from launch_param_builder import load_yaml
from nav2_common.launch import ReplaceString
from pathlib import Path



def generate_launch_description(context):

#declare launch arguments (can be passed in the command line while launching)
    declared_arguments = []

    #sew args
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='sew_',
            description="Prefix for the links and joints of the agv",
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
            "gazebo_standalone",
            default_value='true',
            description="choose gazebo hardware plugin for sew with diff_drive controller, not ros2 control",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "generate_ros2_control_tag",
            default_value='false',
            description="launch the drivers that connect to the real hardware via IP or to gazebo via gazebo_ros2_control -- standalone preferes",
        )
    )

    #igus args
    declared_arguments.append(
        DeclareLaunchArgument(
            "namespace",
            default_value="",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="igus_"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper",
            default_value="none",
            choices=["none", "schmalz_ecbpmi", "ext_dio_gripper"],
            description="Which gripper to attach to the flange",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "hardware_protocol",
            default_value="gazebo",
            choices=["mock_hardware", "gazebo", "cprcanv2", "cri"],
            description="Which hardware protocol or mock hardware should be used",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rebel_version",
            default_value="01",
            choices=["pre", "00", "01"],
            description="Which version of the igus ReBeL to use",
        )
    )
    
    #general launch args
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_manager_name",
            default_value=[LaunchConfiguration("namespace"), "/controller_manager"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Set to "true" if you want to use the gazebo clock, set to "fasle" if you use real hardware.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            choices=["0", "1", "false", "true", "False", "True"],
            description="Whether to start rviz with the launch file",
        )
    )


    tf_prefix = LaunchConfiguration("tf_prefix")
    gazebo_standalone = LaunchConfiguration("gazebo_standalone")
    generate_ros2_control_tag = LaunchConfiguration("generate_ros2_control_tag")
    robot_ip = LaunchConfiguration("robot_ip")
    
    gripper = LaunchConfiguration("gripper")
    prefix = LaunchConfiguration("prefix")
    namespace = LaunchConfiguration("namespace")
    hardware_protocol = LaunchConfiguration("hardware_protocol")
    rebel_version = LaunchConfiguration("rebel_version")

    controller_manager_name = LaunchConfiguration("controller_manager_name")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

#define fixed arguments, files and replace placeholders in yaml files with launch arguments
    moveit_package = "sew_and_igus_moveit_config"
    description_package = "sew_and_igus_description"

#load ros2 control related yaml files
    ros2_controllers_file = PathJoinSubstitution([FindPackageShare(moveit_package),"config", "ros2_controllers_simulation.yaml"])
    ros2_controllers = ReplaceString(
        source_file=ros2_controllers_file,
        replacements={
            "<namespace>": namespace,
            "<prefix>": prefix,
        },
    )


#load robot description related files
    joint_limits_file = PathJoinSubstitution([FindPackageShare(moveit_package),"config","joint_limits.yaml"])
    joint_limits = ReplaceString(
        source_file=joint_limits_file,
        replacements={
            "<namespace>": namespace,
            "<prefix>": prefix,
        },
    )

    robot_description_file = PathJoinSubstitution([FindPackageShare(description_package),"urdf","sew_and_igus_model.urdf.xacro"])
    robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            robot_description_file,
            " prefix:=",
            prefix,
            " hardware_protocol:=",
            hardware_protocol,
            " gripper:=",
            gripper,
            " rebel_version:=",
            rebel_version,
            " tf_prefix:=",
            tf_prefix,
            " gazebo_standalone:=",
            gazebo_standalone,
            " generate_ros2_control_tag:=",
            generate_ros2_control_tag,
            " robot_ip:=",
            robot_ip,
        ]
    )

    robot_description_semantic_file = PathJoinSubstitution([FindPackageShare(moveit_package),"srdf","igus_rebel_6dof.srdf.xacro"])
    robot_description_semantic = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            robot_description_semantic_file,
            " prefix:=",
            prefix,
            " tf_prefix:=",
            tf_prefix,
        ]
    )

    robot_description_kinematics = PathJoinSubstitution([FindPackageShare(moveit_package), "config", "kinematics.yaml"])


# load planning configuration related yaml files
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }

    ompl_file = PathJoinSubstitution([FindPackageShare(moveit_package),"config", "ompl.yaml"])
    ompl_planning_yaml = {"ompl": load_yaml(Path(ompl_file.perform(context)))} ###missing context
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    controllers_file = os.path.join(get_package_share_directory(moveit_package),"config","controllers.yaml")
    controllers = ReplaceString(
        source_file=controllers_file,
        replacements={
            "<namespace>": namespace,
            "<prefix>": prefix,
        },
    )
    controllers_yaml = load_yaml(Path(controllers.perform(context)))   ###missing context


    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Moveit Nodes
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    # Ros2 Control Nodes
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=namespace,
        parameters=[
            ros2_controllers,
            {'use_sim_time': use_sim_time}
        ],#            moveit_args,
    )
    joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["joint_state_broadcaster", "-c", controller_manager_name],
    )
    rebel_6dof_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["rebel_6dof_controller", "-c", controller_manager_name],
    )

    # rviz with moveit configuration
    rviz_config_file = PathJoinSubstitution([FindPackageShare(moveit_package), "rviz", "rviz_moveit.rviz"]) # define path to rviz-config file
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
        ],
    )



    nodes_to_start = [
        control_node,
        joint_state_broadcaster_node,
        rebel_6dof_controller_node,
        move_group_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)