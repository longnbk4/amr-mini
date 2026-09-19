"""
bringup.launch.py
==================
Khởi động diffdrive_node (cầu nối ESP32 <-> ROS2).

LƯU Ý: teleop_keyboard_node CHỦ ĐỘNG KHÔNG được đưa vào launch file này,
vì nó cần đọc bàn phím trực tiếp từ một terminal thật (raw TTY). Chạy launch
ở nền/qua systemd sẽ không cấp cho nó một TTY tương tác. Hãy chạy
diffdrive_node qua launch (hoặc service), rồi chạy teleop_keyboard_node
riêng bằng tay trong một terminal khác:

    ros2 launch diffdrive_robot bringup.launch.py
    # ở terminal khác:
    ros2 run diffdrive_robot teleop_keyboard_node
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='diffdrive_robot',
            executable='diffdrive_node',
            name='diffdrive_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/serial0',
                'baud_rate': 115200,
                'wheel_diameter_m': 0.065,
                'track_width_m': 0.1415,
                'cmd_vel_timeout': 0.3,
                'control_rate_hz': 20.0,
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'publish_tf': True,
            }],
        ),
    ])
