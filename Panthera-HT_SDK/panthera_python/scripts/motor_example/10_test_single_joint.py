#!/usr/bin/env python3
"""
10 - 逐关节安全测试

依次测试每个关节电机是否正常:
  1. 读取当前位置
  2. 向正方向移动 0.02 圈 (~7.2°)
  3. 返回起点
  4. 向负方向移动 0.02 圈
  5. 返回起点

使用 set_pos_vel_acc (梯形控制), 限速 0.05 转/秒, 加速度 0.02 转/秒².
每步需要按 Enter 确认, Ctrl+C 随时停止。

⚠ 机械臂场景安全提醒:
  - 首次运行时手扶住机械臂
  - 肩关节(2号)和肘关节(3号)承重,停止后仍需保持力矩
  - 如电机失控,直接拔掉调试板 USB 线
"""
import time
from hightorque_serial import HightorqueSerial

PORT = "COM14"
BAUDRATE = 4_000_000
MOTOR_IDS = [4]

TRAVEL = 0.02       # 测试行程 (圈), ~7.2°
VEL_MAX = 0.05      # 最大速度 (转/秒)
ACC = 0.02           # 加速度 (转/秒²)
SETTLE_STEPS = 30    # 运动步数 (每步 0.1s → 3s)
SETTLE_DT = 0.1      # 步间隔 (秒)


def move_and_wait(ht: HightorqueSerial, motor_id: int, target: float, label: str):
    """发送梯形位置指令并等待到位。"""
    print(f"    → {label}: 目标 {target:+.4f} 圈")
    for i in range(SETTLE_STEPS):
        s = ht.set_pos_vel_acc(motor_id, target, VEL_MAX, ACC)
        if s and i % 5 == 0:
            err = target - s.position
            print(
                f"      [{i + 1:2d}/{SETTLE_STEPS}] "
                f"pos={s.position:+.4f}  vel={s.velocity:+.4f}  "
                f"err={err:+.4f}  mode={s.mode}"
            )
        time.sleep(SETTLE_DT)
    final = ht.read_motor_state(motor_id)
    if final:
        print(f"      到位: pos={final.position:+.4f} 圈")


def test_joint(ht: HightorqueSerial, motor_id: int):
    """测试单个关节: 正向→回→负向→回。"""
    print(f"\n{'=' * 56}")
    print(f"  测试电机 {motor_id}")
    print(f"{'=' * 56}")

    state = ht.read_motor_state(motor_id)
    if state is None:
        print(f"  ✗ 电机 {motor_id} 无响应，跳过")
        return False

    start_pos = state.position
    print(f"  当前位置: {start_pos:+.4f} 圈")
    print(f"  测试范围: {start_pos - TRAVEL:+.4f} ~ {start_pos + TRAVEL:+.4f} 圈")
    print(f"  速度限制: {VEL_MAX} 转/秒, 加速度: {ACC} 转/秒²")

    input("  按 Enter 开始正向测试...")
    move_and_wait(ht, motor_id, start_pos + TRAVEL, "正向")

    input("  按 Enter 返回起点...")
    move_and_wait(ht, motor_id, start_pos, "回零")

    input("  按 Enter 开始负向测试...")
    move_and_wait(ht, motor_id, start_pos - TRAVEL, "负向")

    input("  按 Enter 返回起点...")
    move_and_wait(ht, motor_id, start_pos, "回零")

    print(f"  ✓ 电机 {motor_id} 测试完成")
    return True


if __name__ == "__main__":
    ht = HightorqueSerial(PORT, BAUDRATE)

    print("=" * 56)
    print("  逐关节安全测试")
    print(f"  电机: {MOTOR_IDS}")
    print(f"  行程: ±{TRAVEL} 圈 (±{TRAVEL * 360:.1f}°)")
    print("=" * 56)

    # 先确认所有电机在线
    print("\n--- 通信检查 ---")
    all_ok = True
    for mid in MOTOR_IDS:
        state = ht.read_motor_state(mid)
        if state:
            print(f"  电机 {mid}: pos={state.position:+.4f} 圈, mode={state.mode}, fault=0x{state.fault:02X}")
        else:
            print(f"  电机 {mid}: 无响应!")
            all_ok = False

    if not all_ok:
        print("\n有电机无响应，请检查连接。")
        ht.close()
        exit(1)

    results = {}
    try:
        for mid in MOTOR_IDS:
            ok = test_joint(ht, mid)
            results[mid] = ok
    except KeyboardInterrupt:
        print("\n\n紧急停止!")

    # 停止所有电机
    for mid in MOTOR_IDS:
        ht.stop(mid)

    print(f"\n{'=' * 56}")
    print("  测试结果:")
    for mid in MOTOR_IDS:
        status = "通过" if results.get(mid) else "未完成"
        print(f"    电机 {mid}: {status}")
    print(f"{'=' * 56}")

    ht.close()
