#!/usr/bin/env python3
"""
02 - 位置控制 (正弦波)

使用 set_position 让电机沿正弦轨迹往复运动。
振幅 0.15 圈 (~1 rad), 频率 0.1 Hz。
Ctrl+C 退出并停止电机。

⚠ 机械臂安全提醒:
  - set_position 以最大速度/力矩运动, 在机械臂上可能很危险!
  - 建议改用 10_test_single_joint.py (梯形控制, 有限速保护)
  - 如需使用本脚本, 请先降低 AMPLITUDE 并只控制单个电机
"""
import math
import time
from hightorque_serial import HightorqueSerial

PORT = "COM14"
BAUDRATE = 4_000_000
MOTOR_IDS = [1]  # 机械臂上建议每次只控制一个关节

AMPLITUDE = 0.15       # 圈 (≈ 1 rad)
FREQUENCY = 0.1        # Hz
LOOP_INTERVAL = 0.05   # 50 ms (串口通信速率限制)
PRINT_EVERY = 20       # 每 20 次循环打印一次 (~1 秒)


if __name__ == "__main__":
    ht = HightorqueSerial(PORT, BAUDRATE)

    print(f"位置正弦控制: 振幅={AMPLITUDE} 圈, 频率={FREQUENCY} Hz")
    print("Ctrl+C 退出\n")

    cnt = 0
    try:
        while True:
            t = time.time()
            target = AMPLITUDE * math.sin(2.0 * math.pi * FREQUENCY * t)

            for mid in MOTOR_IDS:
                state = ht.set_position(mid, target)

                if cnt % PRINT_EVERY == 0 and state:
                    print(
                        f"电机 {mid}  "
                        f"目标: {target:+.4f}  "
                        f"实际: {state.position:+.4f} 圈  "
                        f"速度: {state.velocity:+.4f} 转/秒  "
                        f"模式: {state.mode}"
                    )
            if cnt % PRINT_EVERY == 0:
                print("-" * 60)
            cnt += 1
            time.sleep(LOOP_INTERVAL)
    except KeyboardInterrupt:
        print("\n停止电机...")

    for mid in MOTOR_IDS:
        ht.stop(mid)
    ht.close()
