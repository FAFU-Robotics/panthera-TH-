#!/usr/bin/env python3
"""
05 - DQ 电压控制

使用 set_voltage 输出正弦变化的电压。
振幅 1.0 V, 频率 0.1 Hz。
Ctrl+C 退出并停止电机。

⚠ 机械臂安全提醒:
  - 电压模式是底层控制, 没有位置/速度保护
  - 在机械臂上使用请降低 AMPLITUDE 并手扶机械臂
"""
import math
import time
from hightorque_serial import HightorqueSerial

PORT = "COM14"
BAUDRATE = 4_000_000
MOTOR_IDS = [1]  # 机械臂上建议每次只控制一个关节

AMPLITUDE = 1.0     # V
FREQUENCY = 0.1     # Hz
LOOP_INTERVAL = 0.05
PRINT_EVERY = 20


if __name__ == "__main__":
    ht = HightorqueSerial(PORT, BAUDRATE)

    print(f"DQ 电压正弦控制: 振幅={AMPLITUDE} V, 频率={FREQUENCY} Hz")
    print("Ctrl+C 退出\n")

    cnt = 0
    try:
        while True:
            t = time.time()
            target_volt = AMPLITUDE * math.sin(2.0 * math.pi * FREQUENCY * t)

            for mid in MOTOR_IDS:
                state = ht.set_voltage(mid, target_volt)

                if cnt % PRINT_EVERY == 0 and state:
                    print(
                        f"电机 {mid}  "
                        f"目标电压: {target_volt:+.2f} V  "
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
