    



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