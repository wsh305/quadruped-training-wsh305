import sys
sys.path.insert(0, '/home/ubuntu/unitree_actuator_sdk/lib')

import unitree_actuator_sdk as sdk
import numpy as np
import time

PORT = '/dev/ttyUSB0'
MOTOR_ID = 0
GEAR_RATIO = 6.33
KP = 200.0
KD = 6.0
ZERO_SPACING = 2 * np.pi * 3 / 19   # 单个零点间距 ≈ 56.8°


def wrap_to_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def send_and_wait(serial, cmd, data, dt=0.01):
    serial.sendRecv(cmd, data)
    time.sleep(dt)


def move_to(serial, cmd, data, q_target_out, offset, duration=2.0, dt=0.01):
    q_now_out = data.q / GEAR_RATIO - offset
    delta_out = wrap_to_pi(q_target_out - q_now_out)
    q_rotor_now = data.q
    q_rotor_target = q_rotor_now + delta_out * GEAR_RATIO

    steps = max(1, int(duration / dt))
    for i in range(steps + 1):
        alpha = i / steps
        q_rotor_des = (1 - alpha) * q_rotor_now + alpha * q_rotor_target
        cmd.q = q_rotor_des
        cmd.dq = 0.0
        cmd.tau = 0.0
        cmd.kp = KP / (GEAR_RATIO * GEAR_RATIO)
        cmd.kd = KD / (GEAR_RATIO * GEAR_RATIO)
        send_and_wait(serial, cmd, data, dt)


def hold(serial, cmd, data, hold_time=0.5, dt=0.01):
    cmd.q = data.q
    cmd.dq = 0.0
    cmd.tau = 0.0
    cmd.kp = KP / (GEAR_RATIO * GEAR_RATIO)
    cmd.kd = KD / (GEAR_RATIO * GEAR_RATIO)
    t_end = time.time() + hold_time
    while time.time() < t_end:
        serial.sendRecv(cmd, data)
        time.sleep(dt)


def main():
    serial = sdk.SerialPort(PORT)
    cmd = sdk.MotorCmd()
    data = sdk.MotorData()

    cmd.motorType = sdk.MotorType.GO_M8010_6
    data.motorType = sdk.MotorType.GO_M8010_6
    cmd.mode = sdk.queryMotorMode(sdk.MotorType.GO_M8010_6, sdk.MotorMode.FOC)
    cmd.id = MOTOR_ID
    cmd.kp = 0.0
    cmd.kd = 0.0

    for _ in range(20):
        send_and_wait(serial, cmd, data, 0.02)

    q_power_on_out = data.q / GEAR_RATIO
    n_jumps = round(q_power_on_out / ZERO_SPACING)
    offset = n_jumps * ZERO_SPACING

    print(f"上电读数(输出侧): {np.rad2deg(q_power_on_out):.3f}°")
    print(f"自动选择零点间距个数 n = {n_jumps}")
    print(f"offset = {np.rad2deg(offset):.3f}°")

    q_show = data.q / GEAR_RATIO - offset
    print(f"对齐后软件显示位置: {np.rad2deg(q_show):.3f}°")

    print("=== 移动到软件零点，请标记当前物理位置 ===")
    move_to(serial, cmd, data, 0.0, offset, duration=2.0)
    hold(serial, cmd, data, hold_time=0.5)
    input("标记完成后按回车继续...")

    print("=== 键盘输入目标角度，观察是否跳变 ===")
    while True:
        s = input("输入目标角度(度)，q 退出: ").strip()
        if s.lower() == 'q':
            break
        try:
            q_deg = float(s)
        except ValueError:
            continue

        q_target_out = np.deg2rad(q_deg)
        move_to(serial, cmd, data, q_target_out, offset, duration=2.0)
        hold(serial, cmd, data, hold_time=0.8)

        q_show = data.q / GEAR_RATIO - offset
        print(f"目标: {q_deg:.2f}° | 实际: {np.rad2deg(q_show):.3f}° | 误差: {np.rad2deg(q_show) - q_deg:+.3f}°")


if __name__ == "__main__":
    main()