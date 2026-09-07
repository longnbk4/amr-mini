#!/usr/bin/env python3
import serial
import threading
import sys
import time
import re

# ==================== CẤU HÌNH ====================
PORT = '/dev/serial0'
BAUD_RATE = 115200
DEFAULT_SPEED = 180
MIN_SPEED = 80
MAX_SPEED = 255

# ==================== BIẾN ====================
current_left = 0
current_right = 0
is_running = True
status_text = "DỪNG"

tele = {
    "rpmL": 0.0, "rpmR": 0.0,
    "x": 0.0, "y": 0.0, "theta": 0.0,
    "ticksL": 0, "ticksR": 0,
}
tele_lock = threading.Lock()

# Chỉ nhận dòng đúng format: E,num,num,num,num,num,num,num
E_PATTERN = re.compile(
    r'^E,'
    r'(-?\d+),'          # ticksL
    r'(-?\d+),'          # ticksR
    r'(-?\d+(?:\.\d+)?),'  # rpmL
    r'(-?\d+(?:\.\d+)?),'  # rpmR
    r'(-?\d+(?:\.\d+)?),'  # x
    r'(-?\d+(?:\.\d+)?),'  # y
    r'(-?\d+(?:\.\d+)?)$'  # theta
)

def clear_line():
    sys.stdout.write("\r" + " " * 110 + "\r")
    sys.stdout.flush()

def print_status():
    with tele_lock:
        t = tele.copy()
    line = (
        f"[{status_text:8}] PWM L:{current_left:4d} R:{current_right:4d} | "
        f"RPM L:{t['rpmL']:6.1f} R:{t['rpmR']:6.1f} | "
        f"x:{t['x']:7.3f} y:{t['y']:7.3f} θ:{t['theta']:7.1f}° | "
        f"Ticks {t['ticksL']:6d}/{t['ticksR']:6d}"
    )
    clear_line()
    sys.stdout.write(line)
    sys.stdout.flush()

def parse_telemetry(line):
    """Trả về dict nếu hợp lệ, None nếu là rác"""
    m = E_PATTERN.match(line.strip())
    if not m:
        return None
    try:
        return {
            "ticksL": int(m.group(1)),
            "ticksR": int(m.group(2)),
            "rpmL":   float(m.group(3)),
            "rpmR":   float(m.group(4)),
            "x":      float(m.group(5)),
            "y":      float(m.group(6)),
            "theta":  float(m.group(7)),
        }
    except (ValueError, IndexError):
        return None

def read_from_esp(ser):
    global tele
    bad_count = 0

    while is_running:
        try:
            if ser.in_waiting > 0:
                raw = ser.readline().decode('utf-8', errors='ignore').strip()
                if not raw:
                    continue

                if raw == "OK":
                    continue

                data = parse_telemetry(raw)
                if data is not None:
                    with tele_lock:
                        tele.update(data)
                    bad_count = 0
                    print_status()
                else:
                    # Dòng rác — bỏ qua, không in lỗi liên tục
                    bad_count += 1
                    if bad_count >= 20:
                        # Xả buffer nếu nhiễu quá nhiều
                        try:
                            ser.reset_input_buffer()
                        except Exception:
                            pass
                        bad_count = 0
            else:
                time.sleep(0.005)
        except Exception:
            # Không để thread chết vì 1 lỗi đọc
            time.sleep(0.02)

def keep_alive_sender(ser):
    while is_running:
        try:
            packet = f"V,{current_left},{current_right}\n"
            ser.write(packet.encode())
            time.sleep(0.05)
        except Exception:
            break

def set_speed(left, right, label):
    global current_left, current_right, status_text
    current_left = int(left)
    current_right = int(right)
    status_text = label
    clear_line()
    print(f"→ {label}  (PWM {current_left}, {current_right})")
    print_status()

def change_default(delta):
    global DEFAULT_SPEED
    DEFAULT_SPEED = max(MIN_SPEED, min(MAX_SPEED, DEFAULT_SPEED + delta))
    clear_line()
    print(f"→ DEFAULT_SPEED = {DEFAULT_SPEED}")
    if status_text == "TIẾN":
        set_speed(DEFAULT_SPEED, DEFAULT_SPEED, "TIẾN")
    elif status_text == "LÙI":
        set_speed(-DEFAULT_SPEED, -DEFAULT_SPEED, "LÙI")
    elif status_text == "TRÁI":
        set_speed(-DEFAULT_SPEED, DEFAULT_SPEED, "TRÁI")
    elif status_text == "PHẢI":
        set_speed(DEFAULT_SPEED, -DEFAULT_SPEED, "PHẢI")
    else:
        print_status()

def main():
    global current_left, current_right, is_running, status_text

    try:
        ser = serial.Serial(PORT, BAUD_RATE, timeout=0.1)
        # Xả rác cũ trong buffer khi mới mở
        time.sleep(0.1)
        ser.reset_input_buffer()
        print(f"[*] Đã kết nối ESP32 qua {PORT}")
    except Exception as e:
        print(f"[!] Không mở được {PORT}: {e}")
        sys.exit(1)

    t_read = threading.Thread(target=read_from_esp, args=(ser,), daemon=True)
    t_read.start()
    t_send = threading.Thread(target=keep_alive_sender, args=(ser,), daemon=True)
    t_send.start()

    print()
    print("=" * 60)
    print("  ĐIỀU KHIỂN ROBOT + ODOMETRY")
    print("=" * 60)
    print("  W / S / A / D   : Tiến / Lùi / Xoay trái / Xoay phải")
    print("  X               : Dừng")
    print("  R               : Reset odometry")
    print("  + / -           : Tăng / giảm tốc độ mặc định (±10)")
    print("  180             : Đặt cùng tốc độ 2 bánh")
    print("  180,160         : Đặt riêng L,R")
    print("  H               : Hiện lại hướng dẫn")
    print("  Ctrl+C          : Thoát")
    print("=" * 60)
    print(f"  DEFAULT_SPEED = {DEFAULT_SPEED}")
    print()

    try:
        while True:
            cmd = input("").strip().upper()

            if cmd == 'W':
                set_speed(DEFAULT_SPEED, DEFAULT_SPEED, "TIẾN")
            elif cmd == 'S':
                set_speed(-DEFAULT_SPEED, -DEFAULT_SPEED, "LÙI")
            elif cmd == 'A':
                set_speed(-DEFAULT_SPEED, DEFAULT_SPEED, "TRÁI")
            elif cmd == 'D':
                set_speed(DEFAULT_SPEED, -DEFAULT_SPEED, "PHẢI")
            elif cmd == 'X':
                set_speed(0, 0, "DỪNG")
            elif cmd == 'R':
                try:
                    ser.write(b"R\n")
                except Exception:
                    pass
                clear_line()
                print("→ Đã gửi RESET ODOMETRY")
                time.sleep(0.15)
                print_status()
            elif cmd in ('+', '='):
                change_default(+10)
            elif cmd == '-':
                change_default(-10)
            elif cmd == 'H':
                clear_line()
                print("  W/S/A/D tiến/lùi/trái/phải | X dừng | R reset | +/- tốc độ")
                print_status()
            elif ',' in cmd:
                try:
                    a, b = cmd.split(',', 1)
                    set_speed(int(a.strip()), int(b.strip()), "TÙY CHỈNH")
                except Exception:
                    clear_line()
                    print("[!] Sai cú pháp. Ví dụ: 180,160")
                    print_status()
            elif cmd.lstrip('-').isdigit():
                try:
                    v = int(cmd)
                    set_speed(v, v, "TÙY CHỈNH")
                except Exception:
                    clear_line()
                    print("[!] Số không hợp lệ")
                    print_status()
            elif cmd != '':
                clear_line()
                print("[!] Lệnh không hợp lệ — gõ H để xem hướng dẫn")
                print_status()

    except KeyboardInterrupt:
        clear_line()
        print("\n[*] Dừng khẩn cấp...")
        is_running = False
        try:
            ser.write(b"V,0,0\n")
            time.sleep(0.15)
            ser.close()
        except Exception:
            pass
        print("[*] Đã thoát.")

if __name__ == '__main__':
    main()
