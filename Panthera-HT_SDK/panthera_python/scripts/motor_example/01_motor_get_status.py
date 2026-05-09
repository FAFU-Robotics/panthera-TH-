#!/usr/bin/env python3
"""
01 - 读取电机状态

循环读取并打印所有电机的位置、速度、力矩、模式和故障码。
Ctrl+C 退出。

⚠ 机械臂安全提醒: 本脚本只读取状态, 不发送控制指令, 可安全运行。
  建议每次控制前先运行此脚本确认所有电机在线。
"""
import time
from hightorque_serial import HightorqueSerial

PORT = "COM14"
BAUDRATE = 4_000_000
MOTOR_IDS = [1,2,3,4,5,6,7]  # 根据实际连接的电机修改


if __name__ == "__main__":
    ht = HightorqueSerial(PORT, BAUDRATE)

    print(f"读取 {len(MOTOR_IDS)} 个电机状态 (Ctrl+C 退出)\n")

    try:
        while True:
            for mid in MOTOR_IDS:
                state = ht.read_motor_state(mid)
                if state is None:
                    print(f"电机 {mid}: 无响应")
                    continue
                print(
                    f"电机 {mid}  "
                    f"位置: {state.position:+.4f} 圈  "
                    f"速度: {state.velocity:+.4f} 转/秒  "
                    f"力矩: {state.torque:+6d} (raw)  "
                    f"模式: {state.mode}  "
                    f"故障: 0x{state.fault:02X}"
                )
            print("-" * 72)
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n已退出")

    for mid in MOTOR_IDS:
        ht.stop(mid)
    ht.close()
