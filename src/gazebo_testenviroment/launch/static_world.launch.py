from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import ThisLaunchFileDir
from launch_ros.substitutions import FindPackageShare, FindPackage


def generate_launch_description():

    #declare launch arguments
    declared_arguments = []
    declared_arguments.append(
    DeclareLaunchArgument('gui', default_value='true',
                            description='Set to "false" to run headless.'))
    
    declared_arguments.append(
    DeclareLaunchArgument('server', default_value='true',
                            description='Set to "false" not to run gzserver.'))
    
    declared_arguments.append(
    DeclareLaunchArgument('world', default_value='src/gazebo_testenviroment/worlds/static_world_2404.world',
                            description='Specify the world which should be loaded in Gazebo'))
    
    #static_world_2404

    #init launch arguments, transfer to variables
    world = LaunchConfiguration('world')


    #define nodes to launch
    gzserver_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch']), '/gzserver.launch.py']),
        condition=IfCondition(LaunchConfiguration('server')),
        launch_arguments={
            'world': world,
        }.items(),
    )

    gzclient_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
           [PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch']), '/gzclient.launch.py']),
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    nodes_to_start = [gzclient_node, gzserver_node]


    return LaunchDescription(declared_arguments + nodes_to_start)