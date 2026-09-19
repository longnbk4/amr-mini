# diffdrive_robot (ROS2)

Package hiện thực phần **giữa** của sơ đồ:

```
Teleop (bàn phím) -> /cmd_vel -> diffdrive_node -> UART "V,<l_rpm>,<r_rpm>" -> ESP32
                                        ^
                                        |__ đọc "E,..." từ ESP32 -> /odom + tf
```

## Cài đặt

```bash
sudo apt install python3-pip
pip3 install pyserial

# copy thư mục này vào workspace ROS2 của bạn
cp -r diffdrive_robot ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select diffdrive_robot
source install/setup.bash
```

## Chạy

Terminal 1 — chạy cầu nối UART <-> ROS2 (dùng launch file, khớp thông số
`WHEEL_DIAMETER_M` / `TRACK_WIDTH_M` trong `main.c`):

```bash
ros2 launch diffdrive_robot bringup.launch.py
```

Terminal 2 — chạy teleop (**bắt buộc chạy tay trong terminal thật**, không
qua launch, vì cần đọc bàn phím raw trực tiếp — xem giải thích trong
`launch/bringup.launch.py`):

```bash
ros2 run diffdrive_robot teleop_keyboard_node
```

Kiểm tra nhanh không cần bàn phím:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}" -r 10
ros2 topic echo /odom
```

## ⚠️ Điều quan trọng cần biết trước khi test tốc độ thật

Firmware ESP32 (`main.c`) hiện tại **chưa có PID**. Lệnh `V,<l>,<r>` đang
được hiểu là **giá trị PWM thô** (-255..255) đưa thẳng vào `set_motor_speed()`,
**không phải RPM**.

Node `diffdrive_node` trong package này gửi đúng ý nghĩa **RPM setpoint**
theo thiết kế cuối (sau khi bạn thêm vòng PID nhận `target RPM` → xuất
`PWM`). Trước khi ESP32 có PID:

- Giao tiếp ROS2 <-> ESP32 (topic, service, đọc telemetry, tf, odom) **hoạt
  động và test được bình thường**.
- Nhưng **tốc độ thực tế của robot sẽ sai** (vì số RPM tính ra bị ESP32
  hiểu nhầm là PWM) — chỉ dùng giai đoạn này để kiểm tra luồng giao tiếp,
  chưa dùng để tune tốc độ/hành vi thật.
- Sau khi thêm PID vào `main.c` (nhánh xử lý lệnh `'V'`: dùng 2 số nhận
  được làm setpoint RPM cho vòng PID trái/phải thay vì gán thẳng PWM),
  mọi thứ ở tầng ROS2 không cần sửa gì thêm.

## Ngưỡng phát hiện "nhả phím" (teleop_keyboard_node)

Terminal không có sự kiện key-up thật — chỉ có ký tự lặp lại (auto-repeat)
khi giữ phím. `INITIAL_GRACE` (0.6s) chờ đủ lâu cho lần gõ đầu tiên (qua
giai đoạn delay lặp phím ban đầu của OS), `STEADY_TIMEOUT` (0.12s) dùng sau
khi đã xác nhận đang giữ phím thật — dừng nhanh gần như ngay khi nhả tay.
Nếu terminal của bạn bị khựng/dừng-giả, chỉnh 2 hằng số này trong
`teleop_keyboard_node.py`.

## An toàn — 2 lớp watchdog độc lập

1. **teleop_keyboard_node**: tự phát hiện nhả phím, publish `Twist(0,0)`.
2. **diffdrive_node**: có watchdog riêng (`cmd_vel_timeout`, mặc định 0.3s)
   — nếu không nhận `/cmd_vel` mới trong khoảng đó (teleop crash, SSH rớt,
   bạn dùng node điều khiển khác bị treo...), tự động gửi lệnh dừng xuống
   ESP32. Hai lớp này độc lập nhau — hỏng 1 lớp vẫn còn lớp kia.

Firmware ESP32 vẫn giữ nguyên `COMMAND_TIMEOUT_MS = 10000` làm lớp an toàn
thứ 3 ở tầng thấp nhất (phòng khi mất luôn kết nối UART).

## Tham số của diffdrive_node

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `serial_port` | `/dev/serial0` | Cổng UART tới ESP32 |
| `baud_rate` | `115200` | Khớp `main.c` |
| `wheel_diameter_m` | `0.065` | Khớp `WHEEL_DIAMETER_M` |
| `track_width_m` | `0.1415` | Khớp `TRACK_WIDTH_M` |
| `cmd_vel_timeout` | `0.3` | Giây, watchdog dừng khi mất `/cmd_vel` |
| `control_rate_hz` | `20.0` | Tần số gửi lệnh UART xuống ESP32 |
| `publish_tf` | `true` | Có broadcast tf `odom -> base_link` không |
