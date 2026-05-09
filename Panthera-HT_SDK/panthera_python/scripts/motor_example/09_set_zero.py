#!/usr/bin/env python3
"""
09 - 重置零点

1. 先读取并打印当前电机状态 (5 秒)
2. 执行 reset_zero (设为新零点)
3. save_config (保存到 Flash)
4. motor_reset (软重启)
5. 再次读取状态，确认零位已更新

⚠ 机械臂安全提醒:
  - 重置零点会改变电机坐标系, 影响所有后续位置指令!
  - 请确保机械臂在正确的零位姿态后再执行
  - motor_reset 会导致电机短暂失力, 肩/肘关节可能因重力下坠
  - 执行前请用手扶住机械臂
"""
import time
from hightorque_serial import HightorqueSerial

PORT = "COM14"
BAUDRATE = 4_000_000
MOTOR_IDS = [3]  # 建议每次只重置一个关节的零点



def print_states(ht, label=""):
    if label:
        print(f"\n{label}")
    for mid in MOTOR_IDS:
        state = ht.read_motor_state(mid)
        if state is None:
            print(f"  电机 {mid}: 无响应")
            continue
        print(
            f"  电机 {mid}  "
            f"位置: {state.position:+.4f} 圈  "
            f"速度: {state.velocity:+.4f} 转/秒  "
            f"力矩: {state.torque:+6d}  "
            f"模式: {state.mode}  "
            f"故障: 0x{state.fault:02X}"
        )


if __name__ == "__main__":
    ht = HightorqueSerial(PORT, BAUDRATE)

    print("=== 重置零点流程 ===\n")

    # --- 阶段 1: 读取当前状态 ---
    print("--- 阶段 1: 当前状态 (5 次) ---")
    for i in range(5):
        print_states(ht, f"[{i + 1}/5]")
        time.sleep(0.5)

    # --- 阶段 2: 设零 ---
    input("\n确认要将当前位置设为零点吗? 按 Enter 继续, Ctrl+C 取消...")

    for mid in MOTOR_IDS:
        print(f"\n电机 {mid}: reset_zero → {ht.reset_zero(mid)}")
        time.sleep(0.5)
        print(f"电机 {mid}: save_config → {ht.save_config(mid)}")
        time.sleep(0.5)
        print(f"电机 {mid}: motor_reset → {ht.motor_reset(mid)}")

    print("\n等待电机重启...")
    time.sleep(2.0)

    # --- 阶段 3: 验证 ---
    print("\n--- 阶段 3: 重启后状态 ---")
    for i in range(10):
        print_states(ht, f"[{i + 1}/10]")
        time.sleep(0.5)

    ht.close()
    print("\n=== 完成 ===")
