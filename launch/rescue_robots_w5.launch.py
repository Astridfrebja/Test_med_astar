import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'multi_robot_challenge_23'

    # --- 1. FILSTI-DEFINISJONER ---
    world_file_path = os.path.join(get_package_share_directory(package_name), 'worlds', 'dat160_w5.world')
    map_file_path = os.path.join(get_package_share_directory(package_name), 'maps', 'map_dat160_w5.yaml')
    rviz_config_file_path = os.path.join(get_package_share_directory(package_name), 'rviz', 'model.rviz')

    # --- 2. ROBOTKONFIGURASJON ---
    robot_namespaces = ['tb3_0', 'tb3_1']
    robot_positions = [
        ['0.0', '-0.5', '0.0'], 
        ['0.0', '-2.5', '0.0'],
    ]
    robot_yaws = ['0.0', '0.0']
    
    # Deklarasjoner for bakoverkompatibilitet (uendret, OK)
    first_tb3 = robot_namespaces[0]
    second_tb3 = robot_namespaces[1] if len(robot_namespaces) > 1 else None
    first_tb3_pos = robot_positions[0]
    second_tb3_pos = robot_positions[1] if len(robot_positions) > 1 else None
    first_tb3_yaw = robot_yaws[0]
    second_tb3_yaw = robot_yaws[1] if len(robot_yaws) > 1 else '0.0'

    # --- 3. ARGUMENTER ---
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # --- 4. LANSERING AV HOVEDKOMPONENTER ---
    
    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={'world': world_file_path}.items()
    )

    # Map Server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{"yaml_filename": map_file_path, "topic_name": "map", "frame_id": "map"}],
    )
    
    # Lifecycle Manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ["map_server"]}]
    )

    # TF world -> map
    tf_world_to_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_world_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        output='screen'
    )

    # --- 5. ROBOTER (Inkluderer spawn_robot.launch.py) ---
    tb3_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(package_name), 'launch'), '/spawn_robot.launch.py']),
        launch_arguments={
            'namespace': first_tb3,
            'x': first_tb3_pos[0],
            'y': first_tb3_pos[1],
            'yaw': first_tb3_yaw,
            'use_sim_time': use_sim_time, # Legg til use_sim_time
        }.items()
    )

    tb3_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(package_name), 'launch'), '/spawn_robot.launch.py']),
        launch_arguments={
            'namespace': second_tb3,
            'x': second_tb3_pos[0],
            'y': second_tb3_pos[1],
            'yaw': second_tb3_yaw,
            'use_sim_time': use_sim_time, # Legg til use_sim_time
        }.items()
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Robot Kontroller-Noder
    robot1_node = Node(
        package=package_name,
        executable='robot_node',
        name='robot_controller_tb3_0',
        namespace=first_tb3,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    robot2_node = Node(
        package=package_name,
        executable='robot_node',
        name='robot_controller_tb3_1',
        namespace=second_tb3,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # --- 6. BYGG LAUNCH DESCRIPTION (FJERNER DUPLISERT TF) ---
    # Fjern 'tf_map_to_odom_nodes' som forårsaket konflikter/duplisering
    launch_nodes = [
        sim_time_arg,
        gazebo,
        map_server,
        lifecycle_manager,
        tf_world_to_map,  # world -> map
        tb3_0,
        tb3_1,
        rviz_node,
        robot1_node,
        robot2_node,
    ]
    
    return LaunchDescription(launch_nodes)