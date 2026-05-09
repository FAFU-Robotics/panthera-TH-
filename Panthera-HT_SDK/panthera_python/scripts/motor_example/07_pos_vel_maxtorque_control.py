#!/usr/bin/env python3
"""
07 - 位置+速度+最大力矩控制

使用 set_pos_vel_tqe 在两个位置间每 3 秒切换。
速度 0.08 转/秒 (≈ 0.5 rad/s), 最大力矩原始值 3。
Ctrl+C 退出并停止电机。

⚠ 机械臂安全提醒:
  - 本脚本有速度限制, 相对安全
  - 在机械臂上使用前请先降低 POS_B 为小值 (如 0.02)
  - 建议每次只控制一个关节
"""
import math
import time
from hightorque_serial import HightorqueSerial

PORT = "COM14"
BAUDRATE = 4_000_000
MOTOR_IDS = [1]  # 机械臂上建议每次只控制一个关节

POS_A = 0.0             # 圈
POS_B = 0.16            # 圈 (≈ 1 rad)
VEL = 0.08              # 转/秒 (≈ 0.5 rad/s)
TQE_MAX = 3             # raw int16
SWITCH_PERIOD = 6        # 秒
LOOP_INTERVAL = 0.05
PRINT_EVERY = 20


if __name__ == "__main__":
    ht = HightorqueSerial(PORT, BAUDRATE)

    print(f"位置+速度+最大力矩控制: {POS_A} ↔ {POS_B} 圈")
    print("Ctrl+C 退出\n")

    cnt = 0
    try:
        while True:
            t = time.time()
            target_pos = POS_B if (math.floor(t) % SWITCH_PERIOD) >= (SWITCH_PERIOD // 2) else POS_A

            for mid in MOTOR_IDS:
                state = ht.set_pos_vel_tqe(mid, target_pos, VEL, TQE_MAX)

                if cnt % PRINT_EVERY == 0 and state:
                    print(
                        f"电机 {mid}  "
                        f"目标: {target_pos:+.4f}  "
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
