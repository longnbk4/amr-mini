import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    package_dir = get_package_share_directory('robot_navigation')

    # Đường dẫn các file cấu hình
    # Đảm bảo các file này đã được copy vào thư mục install thông qua setup.py hoặc CMakeLists.txt
    map_file = os.path.join(package_dir, 'maps', 'maze1.yaml')
    params_file = os.path.join(package_dir, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(package_dir, 'rviz', 'nav2_default_view.rviz')

    use_sim_time = True

    # 1. MAP SERVER
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': map_file},
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # 2. AMCL
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # 3. PLANNER SERVER
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # 4. CONTROLLER SERVER
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # 5. BEHAVIOR SERVER
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # 6. BT NAVIGATOR
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # 7. CÁC NODE PHỤ TRỢ (SMOOTHER, VELOCITY, SAVER)
    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    map_saver = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    # 8. LIFECYCLE MANAGER DUY NHẤT (QUAN TRỌNG)
    # Gộp tất cả vào một manager và để autostart=True để hệ thống tự kích hoạt
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'map_server',
                'amcl',
                'planner_server',
                'controller_server',
                'smoother_server',
                'behavior_server',
                'bt_navigator',
                'velocity_smoother'
            ]
        }]
    )

    # 9. RVIZ
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        map_server,
        amcl,
        planner_server,
        controller_server,
        behavior_server,
        bt_navigator,
        smoother_server,
        velocity_smoother,
        map_saver,
        lifecycle_manager,
        rviz_node
    ])