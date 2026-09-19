#!/usr/bin/env python3
"""
diffdrive_node
===============
Cầu nối ROS2 <-> ESP32 qua UART, đúng theo sơ đồ:

    /cmd_vel (Twist: linear.x + angular.z)
        -> Vl = linear.x - angular.z*(TRACK_WIDTH/2)
           Vr = linear.x + angular.z*(TRACK_WIDTH/2)
        -> đổi sang RPM
        -> gửi UART: "V,<left_rpm>,<right_rpm>\\n"

Đồng thời đọc dòng telemetry "E,ticksL,ticksR,rpmL,rpmR,x,y,theta_deg\\n"
ESP32 gửi lên, publish nav_msgs/Odometry lên /odom + broadcast tf
odom -> base_link.

AN TOÀN — watchdog độc lập:
    Node này có timer riêng kiểm tra "đã bao lâu chưa nhận /cmd_vel mới".
    Nếu quá `cmd_vel_timeout` giây, tự động gửi lệnh dừng — bất kể node
    teleop có đang chạy đúng, bị crash, hay mất kết nối SSH hay không.
    Đây là lớp an toàn thứ 2, tách biệt khỏi cơ chế "nhả phím tự dừng"
    bên teleop_keyboard_node.

LƯU Ý QUAN TRỌNG (đọc trước khi test):
    Firmware ESP32 hiện tại (main.c) CHƯA có PID — lệnh "V,l,r" đang được
    hiểu là giá trị PWM thô (-255..255) đưa thẳng vào set_motor_speed(),
    KHÔNG PHẢI RPM. Node này gửi đúng ý nghĩa "RPM setpoint" theo thiết kế
    cuối cùng (sau khi bạn thêm vòng PID trong main.c). Trước khi ESP32 có
    PID, robot sẽ CHẠY SAI TỐC ĐỘ nếu test bằng node này (vì RPM tính ra
    bị hiểu nhầm thành PWM) — chỉ dùng để kiểm tra giao tiếp ROS2, chưa
    dùng để tune/đánh giá tốc độ thật.
"""
import math
import re
import threading
import time

import serial

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

TELEMETRY_PATTERN = re.compile(
    r'^E,(-?\d+),(-?\d+),(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?),'
    r'(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?)$'
)


def yaw_to_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class DiffDriveNode(Node):
    def __init__(self):
        super().__init__('diffdrive_node')

        # ---- Tham số (khớp #define trong main.c ESP32) ----
        self.declare_parameter('serial_port', '/dev/serial0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_diameter_m', 0.065)
        self.declare_parameter('track_width_m', 0.1415)
        self.declare_parameter('cmd_vel_timeout', 0.3)   # giây
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)

        self.wheel_circumference = math.pi * float(self.get_parameter('wheel_diameter_m').value)
        self.track_width = float(self.get_parameter('track_width_m').value)
        self.cmd_vel_timeout = float(self.get_parameter('cmd_vel_timeout').value)
        control_rate = float(self.get_parameter('control_rate_hz').value)
        self.control_period = 1.0 / control_rate
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        port = self.get_parameter('serial_port').value
        baud = int(self.get_parameter('baud_rate').value)

        # ---- Trạng thái dùng chung giữa các thread ----
        self._lock = threading.Lock()
        self._last_cmd = (0.0, 0.0)      # (linear_x, angular_z)
        self._last_cmd_time = self.get_clock().now()
        self._serial_lock = threading.Lock()

        # ---- Mở serial ----
        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
            time.sleep(0.15)
            self.ser.reset_input_buffer()
            self.get_logger().info(f'Đã kết nối ESP32 qua {port} @ {baud} baud')
        except Exception as e:
            self.get_logger().error(f'Không mở được cổng serial {port}: {e}')
            raise

        # ---- ROS interfaces ----
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', QoSProfile(depth=10))
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_service(Trigger, 'reset_odometry', self._reset_odometry_cb)

        # ---- Timer điều khiển: tính RPM + gửi UART + watchdog ----
        self.create_timer(self.control_period, self._control_loop)

        # ---- Thread riêng đọc telemetry từ ESP32 (blocking I/O) ----
        self._running = True
        self._rx_thread = threading.Thread(target=self._serial_rx_loop, daemon=True)
        self._rx_thread.start()

        self.get_logger().info(
            f'diffdrive_node sẵn sàng. wheel_circumference={self.wheel_circumference:.4f} m, '
            f'track_width={self.track_width} m, cmd_vel_timeout={self.cmd_vel_timeout}s'
        )

    # ------------------------------------------------------------------
    def _cmd_vel_cb(self, msg: Twist):
        with self._lock:
            self._last_cmd = (msg.linear.x, msg.angular.z)
            self._last_cmd_time = self.get_clock().now()

    def _reset_odometry_cb(self, request, response):
        self._send_line('R')
        response.success = True
        response.message = 'Đã gửi lệnh reset odometry xuống ESP32'
        return response

    # ------------------------------------------------------------------
    def _control_loop(self):
        with self._lock:
            linear_x, angular_z = self._last_cmd
            age_sec = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9

        # --- Watchdog an toàn: không phụ thuộc teleop ---
        if age_sec > self.cmd_vel_timeout:
            linear_x, angular_z = 0.0, 0.0

        v_l = linear_x - angular_z * (self.track_width / 2.0)
        v_r = linear_x + angular_z * (self.track_width / 2.0)

        rpm_l = (v_l / self.wheel_circumference) * 60.0
        rpm_r = (v_r / self.wheel_circumference) * 60.0

        self._send_line(f'V,{int(round(rpm_l))},{int(round(rpm_r))}')

    def _send_line(self, text):
        try:
            with self._serial_lock:
                self.ser.write((text + '\n').encode())
                self.ser.flush()
        except Exception as e:
            self.get_logger().warn(f'Lỗi gửi serial: {e}', throttle_duration_sec=2.0)

    # ------------------------------------------------------------------
    def _serial_rx_loop(self):
        while self._running:
            try:
                raw = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not raw:
                    continue
                m = TELEMETRY_PATTERN.match(raw)
                if not m:
                    continue
                rpm_l = float(m.group(3))
                rpm_r = float(m.group(4))
                x = float(m.group(5))
                y = float(m.group(6))
                theta_deg = float(m.group(7))
                self._publish_odom(x, y, theta_deg, rpm_l, rpm_r)
            except Exception as e:
                self.get_logger().warn(f'Lỗi đọc serial: {e}', throttle_duration_sec=2.0)
                time.sleep(0.05)

    def _publish_odom(self, x, y, theta_deg, rpm_l, rpm_r):
        theta = math.radians(theta_deg)
        qx, qy, qz, qw = yaw_to_quaternion(theta)

        v_l = (rpm_l / 60.0) * self.wheel_circumference
        v_r = (rpm_r / 60.0) * self.wheel_circumference
        vx = (v_l + v_r) / 2.0
        wz = (v_r - v_l) / self.track_width

        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

    def destroy_node(self):
        self._running = False
        self._send_line('X')
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
