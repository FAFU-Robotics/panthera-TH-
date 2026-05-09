#!/usr/bin/env python3
"""
04 - 力矩控制

使用 set_torque 输出正弦变化的力矩。
振幅 0.2 Nm, 频率 0.1 Hz。
Ctrl+C 退出并停止电机。

⚠ 机械臂安全提醒:
  - 力矩模式没有位置限位, 电机会因力矩持续加速!
  - 肩/肘关节(2/3号)有重力负载, 力矩参数需仔细调整
  - 首次使用请降低 AMPLITUDE 并手扶机械臂
"""
import math
import time
from hightorque_serial import HightorqueSerial

PORT = "COM14"
BAUDRATE = 4_000_000
MOTOR_IDS = [1]  # 机械臂上建议每次只控制一个关节

AMPLITUDE = 0.2     # Nm
FREQUENCY = 0.1     # Hz
LOOP_INTERVAL = 0.05
PRINT_EVERY = 20


if __name__ == "__main__":
    ht = HightorqueSerial(PORT, BAUDRATE)

    print(f"力矩正弦控制: 振幅={AMPLITUDE} Nm, 频率={FREQUENCY} Hz")
    print("Ctrl+C 退出\n")

    cnt = 0
    try:
        while True:
            t = time.time()
            target_tqe = AMPLITUDE * math.sin(2.0 * math.pi * FREQUENCY * t)

            for mid in MOTOR_IDS:
                state = ht.set_torque(mid, target_tqe)

                if cnt % PRINT_EVERY == 0 and state:
                    print(
                        f"电机 {mid}  "
                        f"目标力矩: {target_tqe:+.3f} Nm  "
                        f"位置: {state.position:+.4f} 圈  "
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
