import os
import random
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'multi_robot_challenge_23'

    pkg_share = get_package_share_directory(package_name)

    # Worlds and matching maps (PNG+YAML) – keep pairs in-sync
    world_map_pairs = [
        {
            'world': os.path.join(pkg_share, 'worlds', 'dat160_w1.world'),
            'map_yaml': os.path.join(pkg_share, 'maps', 'map_dat160_w1.yaml'),
        },
        {
            'world': os.path.join(pkg_share, 'worlds', 'dat160_w2.world'),
            'map_yaml': os.path.join(pkg_share, 'maps', 'map_dat160_w2.yaml'),
        },
        {
            'world': os.path.join(pkg_share, 'worlds', 'dat160_w3.world'),
            'map_yaml': os.path.join(pkg_share, 'maps', 'map_dat160_w3.yaml'),
        },
        {
            'world': os.path.join(pkg_share, 'worlds', 'dat160_w4.world'),
            'map_yaml': os.path.join(pkg_share, 'maps', 'map_dat160_w4.yaml'),
        },
        {
            'world': os.path.join(pkg_share, 'worlds', 'dat160_w5.world'),
            'map_yaml': os.path.join(pkg_share, 'maps', 'map_dat160_w5.yaml'),
        },
    ]

    # Optional: allow forcing a specific index via arg; otherwise choose randomly
    world_index_arg = DeclareLaunchArgument(
        'world_index', default_value='-1',
        description='Index 0..4 to force a specific world/map. -1 chooses randomly.'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    world_index = LaunchConfiguration('world_index')

    # Resolve world/map choice at launch time
    # We pick randomly in Python here since LaunchConfiguration arithmetic is limited.
    # If a valid index passed via CLI, we use that instead of random.
    selected_pair = None
    try:
        # Environment variable override via LAUNCH_WORLD_INDEX also supported
        env_idx = os.environ.get('LAUNCH_WORLD_INDEX')
        idx = int(env_idx) if env_idx is not None else int(str(world_index.perform(None)))
        if 0 <= idx < len(world_map_pairs):
            selected_pair = world_map_pairs[idx]
    except Exception:
        pass
    if selected_pair is None:
        selected_pair = random.choice(world_map_pairs)

    world_file_path = selected_pair['world']
    map_file_path = selected_pair['map_yaml']
    rviz_config_file_path = os.path.join(pkg_share, 'rviz', 'model.rviz')

    # Robot namespaces (extendable)
    robot_namespaces = ['tb3_0', 'tb3_1']
    # Positions/yaws per robot (match order)
    robot_positions = [
        ['0.0', '-1.0', '0.0'],  # tb3_0
        ['0.0', '1.0', '0.0'],   # tb3_1
    ]
    robot_yaws = ['0.0', '0.0']

    first_tb3 = robot_namespaces[0]
    second_tb3 = robot_namespaces[1]
    first_tb3_pos = robot_positions[0]
    second_tb3_pos = robot_positions[1]
    first_tb3_yaw = robot_yaws[0]
    second_tb3_yaw = robot_yaws[1]

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py'
        ]),
        launch_arguments={'world': world_file_path}.items()
    )

    # Map server + lifecycle manager
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{"yaml_filename": map_file_path, "topic_name": "map", "frame_id": "map"}],
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, {'autostart': True}, {'node_names': ["map_server"]}],
    )

    # Spawn robots
    tb3_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_share, 'launch'), '/spawn_robot.launch.py']),
        launch_arguments={
            'namespace': first_tb3,
            'x': first_tb3_pos[0],
            'y': first_tb3_pos[1],
            'yaw': first_tb3_yaw,
        }.items(),
    )
    tb3_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_share, 'launch'), '/spawn_robot.launch.py']),
        launch_arguments={
            'namespace': second_tb3,
            'x': second_tb3_pos[0],
            'y': second_tb3_pos[1],
            'yaw': second_tb3_yaw,
        }.items(),
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # Robot controllers
    robot1_node = Node(
        package=package_name,
        executable='robot_node',
        name='robot_controller_tb3_0',
        namespace=first_tb3,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    robot2_node = Node(
        package=package_name,
        executable='robot_node',
        name='robot_controller_tb3_1',
        namespace=second_tb3,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # TF: world->map (root) and map->odom for each robot (no namespace on TF nodes)
    tf_world_to_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_world_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        output='screen',
    )
    tf_map_to_odom_nodes = []
    for robot_ns in robot_namespaces:
        tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'tf_map_to_odom_{robot_ns}',
            arguments=['0', '0', '0', '0', '0', '0', 'map', f'{robot_ns}/odom'],
            output='screen',
        )
        tf_map_to_odom_nodes.append(tf_node)

    scoring_node = Node(
        package='scoring',
        executable='scoring',
        name='scoring',
        output='screen',
    )

    # Compose
    launch_nodes = [
        world_index_arg,
        use_sim_time_arg,
        gazebo,
        map_server,
        lifecycle_manager,
        tf_world_to_map,
        tb3_0,
        tb3_1,
        rviz_node,
        robot1_node,
        robot2_node,
        scoring_node,
    ]
    launch_nodes.extend(tf_map_to_odom_nodes)

    return LaunchDescription(launch_nodes)