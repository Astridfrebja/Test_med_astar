#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'multi_robot_challenge_23'

    # --- 1. FILSTIER ---
    world_file_path = os.path.join(
        get_package_share_directory(package_name), 'worlds', 'dat160_w1.world')
    map_file_path = os.path.join(
        get_package_share_directory(package_name), 'maps', 'map_dat160_w1.yaml')
    rviz_config_file_path = os.path.join(
        get_package_share_directory(package_name), 'rviz', 'model.rviz')

    # --- 2. ROBOTOPPSETT ---
    robot_namespaces = ['tb3_0', 'tb3_1']
    robot_positions = [
        ['0.0', '-1.0', '0.0'],  # tb3_0
        ['0.0', '1.0', '0.0'],   # tb3_1
    ]
    robot_yaws = ['0.0', '0.0']

    # --- 3. ARGUMENTER ---
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # --- 4. HOVEDKOMPONENTER ---

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']
        ),
        launch_arguments={'world': world_file_path}.items()
    )

    # Map Server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_file_path,
            'topic_name': 'map',
            'frame_id': 'map'
        }]
    )

    # Lifecycle Manager for map_server
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server'],
            'bond_timeout': 0.0
        }]
    )

    # --- 4b. TF ---
    # world → map
    tf_world_to_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_world_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        output='screen'
    )

    # map → odom for hver robot
    tf_map_to_odom_nodes = []
    for ns in robot_namespaces:
        tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'{ns}_tf_map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', f'{ns}/odom'],
            output='screen'
        )
        tf_map_to_odom_nodes.append(tf_node)

    # --- 5. ROBOTER ---
    tb3_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch'), '/spawn_robot.launch.py'
        ]),
        launch_arguments={
            'namespace': robot_namespaces[0],
            'x': robot_positions[0][0],
            'y': robot_positions[0][1],
            'yaw': robot_yaws[0],
            'use_sim_time': use_sim_time
        }.items()
    )

    tb3_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch'), '/spawn_robot.launch.py'
        ]),
        launch_arguments={
            'namespace': robot_namespaces[1],
            'x': robot_positions[1][0],
            'y': robot_positions[1][1],
            'yaw': robot_yaws[1],
            'use_sim_time': use_sim_time
        }.items()
    )

    # --- 6. RViz ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file_path],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # --- 7. ROBOTKONTROLLERE ---
    robot1_node = Node(
        package=package_name,
        executable='robot_node',
        name='robot_controller_tb3_0',
        namespace='tb3_0',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    robot2_node = Node(
        package=package_name,
        executable='robot_node',
        name='robot_controller_tb3_1',
        namespace='tb3_1',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # --- 8. SAMLE ALT ---
    launch_nodes = [
        sim_time_arg,
        gazebo,
        map_server,
        lifecycle_manager,
        tf_world_to_map,
        *tf_map_to_odom_nodes,
        tb3_0,
        tb3_1,
        rviz_node,
        robot1_node,
        robot2_node
    ]

    return LaunchDescription(launch_nodes)
