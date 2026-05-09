#!/usr/bin/env python3
"""
综合电机控制示例 - 菜单式交互

基于 hightorque_serial 驱动，通过调试板的 ASCII 命令控制电机。
所有示例均可通过菜单选择运行，Ctrl+C 随时返回菜单。

⚠ 机械臂安全提醒:
  - 控制类示例 (2-6) 使用的默认参数针对单电机,
    在机械臂上可能幅度过大, 请先降低参数再运行
  - 推荐使用示例 5 (梯形控制) 或 10/11 号专用脚本
  - 运行前先用示例 1 确认所有电机在线
"""
import math
import time
from hightorque_serial import HightorqueSerial, MotorState

PORT = "COM14"
BAUDRATE = 4_000_000
MOTOR_IDS = [1]  # 机械臂上建议每次只控制一个关节; 多电机: [1, 2, 3]

ht: HightorqueSerial | None = None


def ensure_connected() -> HightorqueSerial:
    global ht
    if ht is None or not ht.ser.is_open:
        ht = HightorqueSerial(PORT, BAUDRATE)
    return ht


def print_state(state: MotorState | None, mid: int):
    if state is None:
        print(f"  电机 {mid}: 无响应")
        return
    print(
        f"  电机 {mid}  "
        f"pos={state.position:+.4f} 圈  "
        f"vel={state.velocity:+.4f} 转/秒  "
        f"tqe={state.torque:+6d}  "
        f"mode={state.mode}  "
        f"fault=0x{state.fault:02X}"
    )


def stop_all():
    conn = ensure_connected()
    for mid in MOTOR_IDS:
        conn.stop(mid)


# ─── 示例 1: 读取状态 ─────────────────────────────────────────────

def example_read_state():
    print("\n" + "=" * 60)
    print("示例 1: 读取电机状态 (Ctrl+C 返回菜单)")
    print("=" * 60)
    conn = ensure_connected()
    while True:
        for mid in MOTOR_IDS:
            print_state(conn.read_motor_state(mid), mid)
        print("-" * 60)
        time.sleep(0.5)


# ─── 示例 2: 正弦位置控制 ─────────────────────────────────────────

def example_position_sin():
    print("\n" + "=" * 60)
    print("示例 2: 正弦位置控制 (振幅 0.15 圈, 0.1 Hz)")
    print("=" * 60)
    conn = ensure_connected()
    amp, freq, dt = 0.15, 0.1, 0.05
    cnt = 0
    while True:
        t = time.time()
        target = amp * math.sin(2.0 * math.pi * freq * t)
        for mid in MOTOR_IDS:
            state = conn.set_position(mid, target)
            if cnt % 20 == 0 and state:
                print(f"  电机 {mid}  目标: {target:+.4f}  实际: {state.position:+.4f} 圈")
        cnt += 1
        time.sleep(dt)


# ─── 示例 3: 速度控制 ────────────────────────────────────────────

def example_velocity():
    print("\n" + "=" * 60)
    print("示例 3: 速度控制 (±0.03 转/秒, 每 3 秒切换)")
    print("=" * 60)
    conn = ensure_connected()
    vel = 0.03
    cnt = 0
    while True:
        t = time.time()
        target_vel = vel if (math.floor(t) % 6) >= 3 else -vel
        for mid in MOTOR_IDS:
            state = conn.set_velocity(mid, target_vel)
            if cnt % 20 == 0 and state:
                print(f"  电机 {mid}  目标vel: {target_vel:+.4f}  实际pos: {state.position:+.4f}")
        cnt += 1
        time.sleep(0.05)


# ─── 示例 4: 力矩控制 ────────────────────────────────────────────

def example_torque():
    print("\n" + "=" * 60)
    print("示例 4: 力矩正弦控制 (0.2 Nm, 0.1 Hz)")
    print("=" * 60)
    conn = ensure_connected()
    amp, freq = 0.2, 0.1
    cnt = 0
    while True:
        t = time.time()
        tqe = amp * math.sin(2.0 * math.pi * freq * t)
        for mid in MOTOR_IDS:
            state = conn.set_torque(mid, tqe)
            if cnt % 20 == 0 and state:
                print(f"  电机 {mid}  力矩: {tqe:+.3f} Nm  pos: {state.position:+.4f}")
        cnt += 1
        time.sleep(0.05)


# ─── 示例 5: 梯形位置控制 (推荐) ──────────────────────────────────

def example_trapezoidal():
    print("\n" + "=" * 60)
    print("示例 5: 梯形位置控制 (set_pos_vel_acc)")
    print("=" * 60)
    conn = ensure_connected()

    state = conn.read_motor_state(MOTOR_IDS[0])
    start_pos = state.position if state else 0.0
    target = start_pos + 0.05
    vel_max = 0.05
    acc = 0.02

    print(f"  起始位置: {start_pos:.4f} 圈")
    print(f"  目标位置: {target:.4f} 圈 (前进 0.05 圈 ≈ 18°)")
    print(f"  最大速度: {vel_max} 转/秒, 加速度: {acc} 转/秒²")
    input("  按 Enter 开始...")

    for i in range(50):
        for mid in MOTOR_IDS:
            s = conn.set_pos_vel_acc(mid, target, vel_max, acc)
            if s and i % 5 == 0:
                print(f"  [{i + 1:2d}] pos={s.position:.4f} vel={s.velocity:.4f} mode={s.mode}")
        time.sleep(0.1)

    print("\n  返回起始位置...")
    for i in range(50):
        for mid in MOTOR_IDS:
            s = conn.set_pos_vel_acc(mid, start_pos, vel_max, acc)
            if s and i % 5 == 0:
                print(f"  [{i + 1:2d}] pos={s.position:.4f} vel={s.velocity:.4f}")
        time.sleep(0.1)
    print("  完成")


# ─── 示例 6: 运控模式 (5参数) ────────────────────────────────────

def example_pos_vel_tqe_kp_kd():
    print("\n" + "=" * 60)
    print("示例 6: 运控模式 (pos+vel+tqe+kp+kd)")
    print("=" * 60)
    conn = ensure_connected()
    amp, freq = 0.5, 0.1
    kp, kd, tqe_ff = 2.0, 0.5, 0.0
    cnt = 0
    while True:
        t = time.time()
        omega = 2.0 * math.pi * freq
        pos = amp * math.sin(omega * t)
        vel = amp * omega * math.cos(omega * t)
        for mid in MOTOR_IDS:
            state = conn.set_pos_vel_tqe_kp_kd(mid, pos, vel, tqe_ff, kp, kd)
            if cnt % 20 == 0 and state:
                print(f"  电机 {mid}  目标: {pos:+.4f}  实际: {state.position:+.4f}")
        cnt += 1
        time.sleep(0.05)


# ─── 示例 7: 电机信息 ────────────────────────────────────────────

def example_motor_info():
    print("\n" + "=" * 60)
    print("示例 7: 电机信息查询")
    print("=" * 60)
    conn = ensure_connected()

    print(f"  CAN 总线状态: {conn.can_status()}")
    print(f"  CAN 配置: {conn.can_config()}")
    print()

    for mid in MOTOR_IDS:
        ver = conn.read_version(mid)
        state = conn.read_motor_state(mid)
        print(f"  电机 {mid}:")
        print(f"    固件版本: {ver or '无响应'}")
        if state:
            print(f"    位置: {state.position:+.4f} 圈")
            print(f"    速度: {state.velocity:+.4f} 转/秒")
            print(f"    力矩: {state.torque:+6d} (raw)")
            print(f"    模式: {state.mode}")
            print(f"    故障: 0x{state.fault:02X}")
        else:
            print("    状态: 无响应")


# ─── 示例 8: 重置零点 ────────────────────────────────────────────

def example_set_zero():
    print("\n" + "=" * 60)
    print("示例 8: 重置零点")
    print("=" * 60)
    conn = ensure_connected()

    print("  当前状态:")
    for mid in MOTOR_IDS:
        print_state(conn.read_motor_state(mid), mid)

    confirm = input("\n  确认重置零点? (y/N): ").strip().lower()
    if confirm != "y":
        print("  已取消")
        return

    for mid in MOTOR_IDS:
        print(f"  电机 {mid}: reset_zero → {conn.reset_zero(mid)}")
        time.sleep(0.5)
        print(f"  电机 {mid}: save_config → {conn.save_config(mid)}")
        time.sleep(0.5)
        print(f"  电机 {mid}: motor_reset → {conn.motor_reset(mid)}")

    print("\n  等待电机重启...")
    time.sleep(2.0)

    print("  重启后状态:")
    for mid in MOTOR_IDS:
        print_state(conn.read_motor_state(mid), mid)


# ─── 主菜单 ──────────────────────────────────────────────────────

EXAMPLES = [
    ("1", "读取电机状态",             example_read_state),
    ("2", "正弦位置控制",             example_position_sin),
    ("3", "速度控制 (±切换)",         example_velocity),
    ("4", "力矩控制 (正弦)",         example_torque),
    ("5", "梯形位置控制 (推荐)",     example_trapezoidal),
    ("6", "运控模式 (5参数)",         example_pos_vel_tqe_kp_kd),
    ("7", "电机信息查询",             example_motor_info),
    ("8", "重置零点",                 example_set_zero),
]


def main():
    print("\n" + "=" * 60)
    print("  高扭矩电机控制 - 综合示例菜单")
    print(f"  端口: {PORT}  波特率: {BAUDRATE}")
    print(f"  电机 ID: {MOTOR_IDS}")
    print("=" * 60)

    while True:
        print("\n可用示例:")
        for num, name, _ in EXAMPLES:
            print(f"  {num}. {name}")
        print("  q. 退出")

        choice = input("\n请选择 (1-8, q): ").strip().lower()
        if choice == "q":
            break

        for num, name, func in EXAMPLES:
            if num == choice:
                try:
                    func()
                except KeyboardInterrupt:
                    print("\n\n  [Ctrl+C] 返回菜单")
                    stop_all()
                except Exception as e:
                    print(f"\n  出错: {e}")
                    import traceback
                    traceback.print_exc()
                    stop_all()
                break
        else:
            print("  无效选择")

    stop_all()
    if ht:
        ht.close()
    print("\n程序结束")


if __name__ == "__main__":
    main()
