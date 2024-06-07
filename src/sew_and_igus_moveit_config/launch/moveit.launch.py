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



def load_yaml(package_name, file_path):
    # TODO make it work with parameter specified package name
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None
    


def generate_launch_description():
    #declare launch arguments (can be passed in the command line while launching)
    declared_arguments = []
    #sew agv
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='sew_',
            description="Prefix for the links and joints from tha SEW AGV",
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
    # igus arm
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
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "controller_manager_name",
    #         default_value=[LaunchConfiguration("namespace"), "/controller_manager"],
    #     )
    # )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rebel_version",
            default_value="01",
            choices=["pre", "00", "01"],
            description="Which version of the igus ReBeL to use",
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

    # general
    declared_arguments.append(
    DeclareLaunchArgument('use_sim_time',
            default_value='true',
            description='Set to "true" if you want to use the gazebo clock, set to "fasle" if you use real hardware.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value='false',
            description="set to true if you want to launch rviz gui (only for testing purposes here!)",
        )
    )

    

    # Initialize Arguments
    tf_prefix = LaunchConfiguration("tf_prefix")
    gazebo_standalone = LaunchConfiguration("standalone_gazebo")
    generate_ros2_control_tag = LaunchConfiguration("generate_ros2_control_tag")
    ros2_control_with_gazebo = LaunchConfiguration("ros2_control_with_gazebo")
    robot_ip = LaunchConfiguration("robot_ip")
    
    gripper = LaunchConfiguration("gripper")
    prefix = LaunchConfiguration("prefix")
    hardware_protocol = LaunchConfiguration("hardware_protocol")
    rebel_version = LaunchConfiguration("rebel_version")

    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")


    #define used packages
    moveit_package = "sew_and_igus_moveit_config"
    description_package = "sew_and_igus_description"


    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package),"urdf","sew_and_igus_model.urdf.xacro"]),
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
            " ros2_control_with_gazebo:=",
            ros2_control_with_gazebo,
            " robot_ip:=",
            robot_ip,
        ]
    )

    robot_description = {"robot_description": robot_description_content} 

    # SRDF configuration (apply collision checking rules)
    robot_description_semantic_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution([FindPackageShare(moveit_package),"srdf", "igus_rebel_6dof.srdf.xacro"]),
            " prefix:=",
            prefix,
            " tf_prefix:=",
            tf_prefix,
        ]
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}


    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(moveit_package), "config", "kinematics.yaml"]
    )


    # Planning Configuration
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }

    ompl_planning_yaml = load_yaml(moveit_package, "config/ompl.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Trajectory Execution Configuration
    controllers_yaml = load_yaml(moveit_package, "config/controllers_fixed_prefix.yaml")
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


    # Start the actual move_group node/action server
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
            {'use_sim_time': use_sim_time},
        ],
    )

    # start arm joint controller
    rebel_6dof_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rebel_6dof_controller", "-c", "/controller_manager"],
    )

    rviz_file = PathJoinSubstitution([FindPackageShare(moveit_package), "rviz", "moveit_rviz.rviz"])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(launch_rviz),
    )


    nodes_to_start = [rebel_6dof_controller_node, move_group_node, rviz_node]


    return LaunchDescription(declared_arguments + nodes_to_start)