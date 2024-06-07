    



    # Octomap parameters
    octomap_params = {
        'frame_id': ParameterValue('map', value_type=str),
        'resolution': ParameterValue(0.025, value_type=float),
        'max_range': ParameterValue(5.0, value_type=float)
    }


    # Octomap server node
    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        parameters=[octomap_params],
        output='screen'
    )

https://answers.ros.org/question/389233/moveit2-add-pointcloud2-to-occupancymapmonitor/
https://github.com/RoboticsYY/moveit_tutorials/blob/ros2/doc/perception_pipeline/launch/obstacle_avoidance_demo.launch.py