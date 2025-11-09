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
    robot_positions = [['0.0', '-1.0', '0.0'], ['0.0', '1.0', '0.0']]
    robot_yaws = ['0.0', '0.0']

    # --- 3. ARGUMENTER ---
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # --- 4. GAZEBO ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={'world': world_file_path}.items()
    )

    # --- 5. MAP SERVER + LIFECYCLE MANAGER ---
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            "yaml_filename": map_file_path,
            "topic_name": "map",
            "frame_id": "map",
            "use_sim_time": True,  # 🟢 Nødvendig for Gazebo-tid
        }],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'bond_timeout': 0.0,   # 🟢 Hindrer heng ved aktivering
            'node_names': ['map_server'],
        }],
    )

    # --- 6. TF WORLD->MAP ---
    tf_world_to_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_world_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        output='screen'
    )

    # --- 7. SPILL INN ROBOTENE ---
    tb3_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch'),
            '/spawn_robot.launch.py'
        ]),
        launch_arguments={
            'namespace': robot_namespaces[0],
            'x': robot_positions[0][0],
            'y': robot_positions[0][1],
            'yaw': robot_yaws[0],
            'use_sim_time': use_sim_time,
        }.items()
    )

    tb3_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch'),
            '/spawn_robot.launch.py'
        ]),
        launch_arguments={
            'namespace': robot_namespaces[1],
            'x': robot_positions[1][0],
            'y': robot_positions[1][1],
            'yaw': robot_yaws[1],
            'use_sim_time': use_sim_time,
        }.items()
    )

    # --- 8. RVIZ ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # --- 9. DEBUG NODE (sjekker om /map kommer ut) ---
    debug_node = Node(
        package='rclpy',
        executable='rclpy',
        name='map_debug_checker',
        output='screen',
        arguments=['-c', """
import rclpy, time
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

rclpy.init()
node = Node('map_debug_checker')
msg_received = False

def cb(msg):
    global msg_received
    msg_received = True
    node.get_logger().info(f'✅ Map mottatt: {msg.info.width}x{msg.info.height}, res={msg.info.resolution}')
    node.destroy_subscription(sub)

sub = node.create_subscription(OccupancyGrid, '/map', cb, 10)

start = time.time()
while rclpy.ok() and time.time() - start < 5 and not msg_received:
    rclpy.spin_once(node, timeout_sec=0.5)

if not msg_received:
    node.get_logger().error('❌ Ingen /map-data mottatt etter 5 sekunder!')
rclpy.shutdown()
"""]
    )

    # --- 10. ROBOTNODER ---
    robot1_node = Node(
        package=package_name,
        executable='robot_node',
        name='robot_controller_tb3_0',
        namespace=robot_namespaces[0],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    robot2_node = Node(
        package=package_name,
        executable='robot_node',
        name='robot_controller_tb3_1',
        namespace=robot_namespaces[1],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # --- 11. BYGG LAUNCHLISTE ---
    launch_nodes = [
        sim_time_arg,
        gazebo,
        map_server,
        lifecycle_manager,
        tf_world_to_map,
        tb3_0,
        tb3_1,
        rviz_node,
        debug_node,
        robot1_node,
        robot2_node,
    ]

    return LaunchDescription(launch_nodes)
