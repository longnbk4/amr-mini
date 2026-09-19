#!/usr/bin/env python3
"""
teleop_keyboard_node
=====================
Teleop bàn phím cho robot diff-drive.

Giữ phím W/S/A/D để robot di chuyển, nhả ra tự dừng (giống "deadman switch"
của joystick). Publish geometry_msgs/Twist lên topic /cmd_vel.

Kỹ thuật phát hiện "giữ/nhả" phím giống bản test UART trực tiếp trước đó:
terminal không có sự kiện key-up thật, nên dựa vào auto-repeat của OS
(ký tự lặp lại liên tục khi giữ phím) + ngưỡng thời gian thích ứng
(INITIAL_GRACE cho lần gõ đầu, STEADY_TIMEOUT sau khi đã thấy lặp lại)
để suy ra lúc nào phím được nhả.

Phím:
    W / S       : tiến / lùi                    (linear.x)
    A / D       : xoay trái / xoay phải tại chỗ  (angular.z)
    X           : dừng ngay
    + / -       : tăng / giảm tốc độ mặc định
    R           : gọi service reset_odometry (do diffdrive_node cung cấp)
    H           : hiện hướng dẫn
    Q / Ctrl+C  : thoát
"""
import sys
import time
import select
import termios
import tty
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

# (linear_sign, angular_sign) cho mỗi phím di chuyển
MOVE_KEYS = {
    'W': (1.0, 0.0),
    'S': (-1.0, 0.0),
    'A': (0.0, 1.0),
    'D': (0.0, -1.0),
}
MOVE_LABELS = {'W': 'TIẾN', 'S': 'LÙI', 'A': 'TRÁI', 'D': 'PHẢI'}

# Xem giải thích chi tiết trong README.md phần "Ngưỡng phát hiện nhả phím"
INITIAL_GRACE = 0.60
STEADY_TIMEOUT = 0.12
PUBLISH_RATE_HZ = 20.0


class TeleopKeyboardNode(Node):
    def __init__(self):
        super().__init__('teleop_keyboard_node')

        self.declare_parameter('linear_speed', 0.2)    # m/s
        self.declare_parameter('angular_speed', 1.0)   # rad/s
        self.declare_parameter('speed_step', 0.02)

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.speed_step = float(self.get_parameter('speed_step').value)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.reset_client = self.create_client(Trigger, 'reset_odometry')

        self.running = True

    def publish_twist(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_pub.publish(msg)

    def call_reset(self):
        if not self.reset_client.service_is_ready():
            self.get_logger().warn(
                'Service reset_odometry chưa sẵn sàng (diffdrive_node đã chạy chưa?)'
            )
            return
        self.reset_client.call_async(Trigger.Request())


def clear_line():
    sys.stdout.write("\r" + " " * 100 + "\r")
    sys.stdout.flush()


def print_status(node, label, lin, ang):
    clear_line()
    sys.stdout.write(
        f"[{label:6}] linear.x={lin:+.2f} m/s  angular.z={ang:+.2f} rad/s   "
        f"(speed={node.linear_speed:.2f} m/s / {node.angular_speed:.2f} rad/s)\n"
    )
    sys.stdout.flush()


def show_help():
    clear_line()
    sys.stdout.write(
        "GIỮ W/S/A/D = tiến/lùi/xoay trái/xoay phải (nhả ra tự dừng) | "
        "X dừng | +/- tốc độ | R reset odometry | Q/Ctrl+C thoát\n"
    )
    sys.stdout.flush()


def keyboard_loop(node: TeleopKeyboardNode):
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    last_move_key = None
    last_key_time = 0.0
    repeat_seen = False
    period = 1.0 / PUBLISH_RATE_HZ

    try:
        tty.setcbreak(fd)
        print()
        print("=" * 65)
        print("  TELEOP BÀN PHÍM -> /cmd_vel  (giữ phím để di chuyển)")
        print("=" * 65)
        show_help()

        while node.running:
            ready, _, _ = select.select([sys.stdin], [], [], period)
            now = time.time()

            if ready:
                ch = sys.stdin.read(1)
                if not ch:
                    continue
                if ch == '\x03':  # Ctrl+C
                    break

                k = ch.upper()

                if k in MOVE_KEYS:
                    if last_move_key != k:
                        last_move_key = k
                        repeat_seen = False
                    else:
                        repeat_seen = True
                    last_key_time = now

                    lin_sign, ang_sign = MOVE_KEYS[k]
                    lin = lin_sign * node.linear_speed
                    ang = ang_sign * node.angular_speed
                    node.publish_twist(lin, ang)
                    print_status(node, MOVE_LABELS[k], lin, ang)

                elif k == 'X':
                    last_move_key = None
                    repeat_seen = False
                    node.publish_twist(0.0, 0.0)
                    print_status(node, 'DỪNG', 0.0, 0.0)

                elif k in ('+', '='):
                    node.linear_speed += node.speed_step
                    node.angular_speed += node.speed_step * 4.0
                    clear_line()
                    print(f"→ linear_speed={node.linear_speed:.2f} m/s, "
                          f"angular_speed={node.angular_speed:.2f} rad/s")

                elif k == '-':
                    node.linear_speed = max(0.02, node.linear_speed - node.speed_step)
                    node.angular_speed = max(0.1, node.angular_speed - node.speed_step * 4.0)
                    clear_line()
                    print(f"→ linear_speed={node.linear_speed:.2f} m/s, "
                          f"angular_speed={node.angular_speed:.2f} rad/s")

                elif k == 'R':
                    node.call_reset()
                    clear_line()
                    print("→ Đã gửi yêu cầu RESET ODOMETRY")

                elif k == 'H':
                    show_help()

                elif k == 'Q':
                    break

            else:
                if last_move_key is not None:
                    threshold = STEADY_TIMEOUT if repeat_seen else INITIAL_GRACE
                    if (now - last_key_time) > threshold:
                        last_move_key = None
                        repeat_seen = False
                        node.publish_twist(0.0, 0.0)
                        print_status(node, 'DỪNG', 0.0, 0.0)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node.publish_twist(0.0, 0.0)  # đảm bảo dừng robot khi thoát


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboardNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        keyboard_loop(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.publish_twist(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
