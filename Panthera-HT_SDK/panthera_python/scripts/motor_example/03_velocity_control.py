#!/usr/bin/env python3
"""
03 - 速度控制

使用 set_velocity 让电机在 ±0.03 转/秒 (≈ ±0.2 rad/s) 之间每 3 秒切换一次。
Ctrl+C 退出并停止电机。

⚠ 机械臂安全提醒:
  - 速度模式没有位置限位, 电机会持续转动!
  - 在机械臂上使用时请降低 VELOCITY 并手扶机械臂
  - 建议只对底座(1号)测试, 肩/肘关节慎用
"""
import math
import time
from hightorque_serial import HightorqueSerial

PORT = "COM14"
BAUDRATE = 4_000_000
MOTOR_IDS = [1]  # 机械臂上建议只控制底座

VELOCITY = 0.03         # 转/秒 (≈ 0.2 rad/s)
SWITCH_PERIOD = 6        # 秒 (前3秒正向, 后3秒反向)
LOOP_INTERVAL = 0.05
PRINT_EVERY = 20


if __name__ == "__main__":
    ht = HightorqueSerial(PORT, BAUDRATE)

    print(f"速度控制: ±{VELOCITY} 转/秒, 每 {SWITCH_PERIOD // 2} 秒切换方向")
    print("Ctrl+C 退出\n")

    cnt = 0
    try:
        while True:
            t = time.time()
            target_vel = VELOCITY if (math.floor(t) % SWITCH_PERIOD) >= (SWITCH_PERIOD // 2) else -VELOCITY

            for mid in MOTOR_IDS:
                state = ht.set_velocity(mid, target_vel)

                if cnt % PRINT_EVERY == 0 and state:
                    print(
                        f"电机 {mid}  "
                        f"目标速度: {target_vel:+.4f}  "
                        f"实际位置: {state.position:+.4f} 圈  "
                        f"实际速度: {state.velocity:+.4f} 转/秒  "
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
