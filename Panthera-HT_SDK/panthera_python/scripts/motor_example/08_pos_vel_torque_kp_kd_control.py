#!/usr/bin/env python3
"""
08 - 运控模式 (pos + vel + torque + kp + kd)

使用 set_pos_vel_tqe_kp_kd 沿正弦轨迹运动。
输出力矩 = (pos偏差)*kp + (vel偏差)*kd + tqe_ff
振幅 0.5 圈 (≈ π rad), 频率 0.1 Hz。
Ctrl+C 退出并停止电机。

⚠ 机械臂安全提醒:
  - 0.5 圈振幅在机械臂上幅度很大 (180°)!
  - 在机械臂上使用前请将 AMPLITUDE 降低到 0.02~0.05
  - Kp/Kd 参数需根据关节负载调整
  - 建议每次只控制一个关节
"""
import math
import time
from hightorque_serial import HightorqueSerial

PORT = "COM14"
BAUDRATE = 4_000_000
MOTOR_IDS = [1]  # 机械臂上建议每次只控制一个关节

AMPLITUDE = 0.5         # 圈 (≈ π rad)
FREQUENCY = 0.1         # Hz
KP = 2.0
KD = 0.5
TQE_FF = 0.0            # 前馈力矩 (Nm)
LOOP_INTERVAL = 0.05
PRINT_EVERY = 20


if __name__ == "__main__":
    ht = HightorqueSerial(PORT, BAUDRATE)

    print(f"运控模式: 振幅={AMPLITUDE} 圈, 频率={FREQUENCY} Hz, Kp={KP}, Kd={KD}")
    print("Ctrl+C 退出\n")

    cnt = 0
    try:
        while True:
            t = time.time()
            omega = 2.0 * math.pi * FREQUENCY
            target_pos = AMPLITUDE * math.sin(omega * t)
            target_vel = AMPLITUDE * omega * math.cos(omega * t)  # d(pos)/dt, 单位: 圈/秒

            for mid in MOTOR_IDS:
                state = ht.set_pos_vel_tqe_kp_kd(mid, target_pos, target_vel, TQE_FF, KP, KD)

                if cnt % PRINT_EVERY == 0 and state:
                    print(
                        f"电机 {mid}  "
                        f"目标pos: {target_pos:+.4f}  "
                        f"目标vel: {target_vel:+.4f}  "
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
