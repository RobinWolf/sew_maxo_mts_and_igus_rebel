# Based on the panda moveit launch file from ros-planning
# https://github.com/ros-planning/moveit_resources/blob/humble/panda_moveit_config/launch/demo.launch.py
# and the moveit_py example
# https://github.com/peterdavidfagan/moveit2_tutorials/blob/moveit_py_motion_planning_python_api_tutorial/doc/examples/motion_planning_python_api/launch/motion_planning_python_api_tutorial.launch.py

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_param_builder import load_yaml
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString


def opaque_test(context, *args, **kwargs):
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

    moveit_package = "sew_and_igus_moveit_config"
    description_package = "sew_and_igus_description"
    
    rviz_file = PathJoinSubstitution(
        [FindPackageShare(moveit_package), "rviz", "moveit_rviz.rviz"]
    )

    # only needed for real hardware --> currently the ros2 control related things are launched in the complete_bringup/simulation.launch.py
    # ros2_controllers_file = PathJoinSubstitution(
    #     [
    #         FindPackageShare(moveit_package),
    #         "config",
    #         "ros2_controllers_simulation.yaml",
    #     ]
    # )
    # ros2_controllers = ReplaceString(
    #     source_file=ros2_controllers_file,
    #     replacements={
    #         "<namespace>": namespace,
    #         "<prefix>": prefix,
    #     },
    # )

    joint_limits_file = PathJoinSubstitution(
        [
            FindPackageShare(moveit_package),
            "config",
            "joint_limits.yaml",
        ]
    )
    joint_limits = ReplaceString(
        source_file=joint_limits_file,
        replacements={
            "<namespace>": namespace,
            "<prefix>": prefix,
        },
    )
    robot_description_file = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "urdf",
            "sew_and_igus_model.urdf.xacro",
        ]
    )
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
    robot_description_semantic_file = PathJoinSubstitution(
        [
            FindPackageShare(moveit_package),
            "srdf",
            "igus_rebel_6dof.srdf.xacro",
        ]
    )
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

    # Requires the os.path.join way instead of PathJoinSubstitution here
    controllers_file = os.path.join(
        get_package_share_directory(moveit_package),
        "config",
        "controllers.yaml",
    )

    controllers = ReplaceString(
        source_file=controllers_file,
        replacements={
            "<namespace>": namespace,
            "<prefix>": prefix,
        },
    )
    controllers_dict = load_yaml(Path(controllers.perform(context)))

    ompl_file = PathJoinSubstitution(
        [
            FindPackageShare(moveit_package),
            "config",
            "ompl.yaml",
        ]
    )
    ompl = {"ompl": load_yaml(Path(ompl_file.perform(context)))}

    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_dict,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    robot_description_kinematics_file = PathJoinSubstitution(
        [
            FindPackageShare(moveit_package),
            "config",
            "kinematics.yaml",
        ]
    )
    # robot_description_kinematics = ReplaceString(
    #     source_file=robot_description_kinematics_file,
    #     replacements={
    #         "<namespace>": namespace,
    #         "<prefix>": prefix,
    #     },
    # )

    planning_pipeline = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        },
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Octomap/ Moveit Perception setup
    octomap_config = {'octomap_frame': 'igus_camera_link_optical', 
                      'octomap_resolution': 0.05,
                      'max_range': 3.0}

    octomap_updater_file = PathJoinSubstitution(
        [
            FindPackageShare(moveit_package),
            "config",
            "sensor_pointcloud.yaml",
        ]
    )
    octomap_updater_config = load_yaml(Path(octomap_updater_file.perform(context)))

    moveit_args_not_concatenated = [
        {"robot_description": robot_description.perform(context)},
        {"robot_description_semantic": robot_description_semantic.perform(context)},
        #load_yaml(Path(robot_description_kinematics.perform(context))),
        load_yaml(Path(joint_limits.perform(context))),
        moveit_controllers,
        planning_scene_monitor_parameters,
        planning_pipeline,
        {
            "publish_robot_description": True,
            "publish_robot_description_semantic": True,
            "publish_geometry_updates": True,
            "publish_state_updates": True,
            "publish_transforms_updates": True,
        },
        ompl,
        octomap_config,
        octomap_updater_config
    ]

    # Concatenate all dictionaries together, else moveitpy won't read all parameters
    moveit_args = dict()
    for d in moveit_args_not_concatenated:
        moveit_args.update(d)



    # Nodes to launch
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=namespace,
        parameters=[
            moveit_args,
            robot_description_kinematics_file,
            {'use_sim_time': use_sim_time}
        ],
    )

    planning_group = {"planning_group": "rebel_6dof"}
    moveit_wrapper_node = Node(
        package="moveit_wrapper",
        executable="moveit_wrapper_node",
        output="screen",
        parameters=[robot_description, robot_description_semantic, robot_description_kinematics_file, planning_group, {'use_sim_time': use_sim_time}],
    )

    # only launch with real hardware -> TODO
    # control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     namespace=namespace,
    #     parameters=[
    #         moveit_args,
    #         ros2_controllers,
    #         {'use_sim_time': use_sim_time}
    #     ],
    # )


    # joint_state_broadcaster_node = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     namespace=namespace,
    #     arguments=["joint_state_broadcaster", "-c", controller_manager_name],
    # )

    # controller for moveit, the controller manager is already up and running when you launch this file from the simulation.launch.py because its launched in the sew_agv_drivers/driver.launch.py
    rebel_6dof_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["rebel_6dof_controller", "-c", controller_manager_name],
    )


    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_file],
        parameters=[
            {"robot_description": robot_description},
            {'use_sim_time': use_sim_time},
            moveit_args,
            robot_description_kinematics_file,
        ],
        condition=IfCondition(launch_rviz),
    )

    return [
        move_group_node,
        moveit_wrapper_node,
        rebel_6dof_controller_node,
        rviz_node,
    ]




def generate_launch_description():
#declare launch arguments (can be passed in the command line while launching)

    #sew args
    tf_prefix_arg = DeclareLaunchArgument(
            "tf_prefix",
            default_value='sew_',
            description="Prefix for the links and joints of the agv",
        )
    robot_ip_arg = DeclareLaunchArgument(
            "robot_ip",
            default_value='TODO',
            description="the IP the real robot can be pinged",
        )
    gazebo_standalone_arg = DeclareLaunchArgument(
            "gazebo_standalone",
            default_value='true',
            description="choose gazebo hardware plugin for sew with diff_drive controller, not ros2 control",
        )
    generate_ros2_control_tag_arg = DeclareLaunchArgument(
            "generate_ros2_control_tag",
            default_value='false',
            description="launch the drivers that connect to the real hardware via IP or to gazebo via gazebo_ros2_control -- standalone preferes",
        )

    #igus args
    namespace_arg = DeclareLaunchArgument(
            "namespace",
            default_value="",
        )
    prefix_arg = DeclareLaunchArgument(
            "prefix",
            default_value="igus_"
        )
    gripper_arg = DeclareLaunchArgument(
            "gripper",
            default_value="none",
            choices=["none", "schmalz_ecbpmi", "ext_dio_gripper"],
            description="Which gripper to attach to the flange",
        )
    hardware_protocol_arg = DeclareLaunchArgument(
            "hardware_protocol",
            default_value="gazebo",
            choices=["mock_hardware", "gazebo", "cprcanv2", "cri"],
            description="Which hardware protocol or mock hardware should be used",
        )
    rebel_version_arg = DeclareLaunchArgument(
            "rebel_version",
            default_value="01",
            choices=["pre", "00", "01"],
            description="Which version of the igus ReBeL to use",
        )
    
    #general launch args
    controller_manager_name_arg = DeclareLaunchArgument(
            "controller_manager_name",
            default_value=[LaunchConfiguration("namespace"), "/controller_manager"],
        )
    use_sim_time_arg = DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Set to "true" if you want to use the gazebo clock, set to "fasle" if you use real hardware.'
        )
    launch_rviz_arg = DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            choices=["0", "1", "false", "true", "False", "True"],
            description="Whether to start rviz with the launch file",
        )

    ld = LaunchDescription()

    ld.add_action(tf_prefix_arg)
    ld.add_action(robot_ip_arg)
    ld.add_action(gazebo_standalone_arg)
    ld.add_action(generate_ros2_control_tag_arg)

    ld.add_action(namespace_arg)
    ld.add_action(prefix_arg)
    ld.add_action(controller_manager_name_arg)
    ld.add_action(gripper_arg)
    ld.add_action(hardware_protocol_arg)
    ld.add_action(rebel_version_arg)

    ld.add_action(launch_rviz_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(controller_manager_name_arg)


    ld.add_action(OpaqueFunction(function=opaque_test))
    return ld

