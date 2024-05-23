from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    sim_package = "gazebo_testenviroment"

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='""',
            description="Prefix for the links and joints in the robot cell",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ros2_control_with_gazebo",
            default_value='false',
            description="add the robot description to gazebo using a ros2_control hardware interface",
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
    DeclareLaunchArgument('world', default_value='src/gazebo_testenviroment/worlds/static_world_2404.world',
                            description='Specify the world which should be loaded in Gazebo'))
    
        
    declared_arguments.append(
    DeclareLaunchArgument('use_sim_time', default_value='true',
                            description='Set to "true" if you want to use the joystick with gazebo, set to "fasle" if you use real hardware.'))
    

    #init launch arguments, transfer to variables
    world = LaunchConfiguration('world')        
    tf_prefix = LaunchConfiguration("tf_prefix")
    ros2_control_with_gazebo = LaunchConfiguration("ros2_control_with_gazebo")      # --> maybe not needed?
    standalone_gazebo = LaunchConfiguration("standalone_gazebo")
    use_sim_time = LaunchConfiguration('use_sim_time')   


    #include gazebo and robot spawning launch
    gazebo_robot_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare(sim_package), 'launch']), "/robot.launch.py"]),
            launch_arguments={
                "tf_prefix": tf_prefix,
                "world": world,
                "ros2_control_with_gazebo": ros2_control_with_gazebo,
                "standalone_gazebo": standalone_gazebo,
            }.items(),
    )

    #include the joystick launch
    joystick_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare(sim_package), 'launch']), "/joystick.launch.py"]),
            launch_arguments={
                "use_sim_time": use_sim_time,
            }.items(),
    )

    nodes_to_start = [gazebo_robot_node, joystick_node]


    return LaunchDescription(declared_arguments + nodes_to_start)