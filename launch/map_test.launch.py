import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    pkg = 'multi_robot_challenge_23'
    map_file = os.path.join(get_package_share_directory(pkg), 'maps', 'map_dat160_w1.yaml')

    # Start map_server immediately
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{"yaml_filename": map_file, "topic_name": "map", "frame_id": "map"}],
    )

    # Start lifecycle_manager after small delay so map_server has time to configure
    lifecycle_manager = TimerAction(
        period=1.5,  # <--- øk/senk om nødvendig
        actions=[Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'autostart': True}, {'bond_timeout': 0.0}, {'node_names': ['map_server']}]
        )]
    )

    # Start debug AFTER lifecycle_manager (litt større delay)
    debug_node = TimerAction(
        period=3.0,  # <--- vente litt lengre enn aktiveringstiden
        actions=[Node(
            package='multi_robot_challenge_23',
            executable='map_debug_checker',
            name='map_debug_checker',
            output='screen'
        )]
    )

    return LaunchDescription([map_server, lifecycle_manager, debug_node])
