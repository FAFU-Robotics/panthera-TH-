#!/usr/bin/env python3
"""
电机控制测试：用梯形控制模式安全地移动电机 1。
按 Ctrl+C 可随时停止电机。
"""
import sys
import time
from hightorque_serial import HightorqueSerial

PORT = "COM14"
BAUDRATE = 4000000
MOTOR_ID = 1

def main():
    print(f"=== 电机控制测试 (电机 {MOTOR_ID}) ===\n")
    ht = HightorqueSerial(PORT, BAUDRATE)

    # 读取当前位置
    state = ht.read_motor_state(MOTOR_ID)
    if not state:
        print("无法读取电机状态，请检查连接")
        ht.close()
        return
    start_pos = state.position
    print(f"当前位置: {start_pos:.4f} 圈")

    # 目标: 从当前位置前进 0.5 圈
    target = start_pos + 0.5
    vel_max = 0.1     # 最大速度 0.1 转/秒 (很慢，安全)
    acc = 0.05         # 加速度 0.05 转/秒^2

    print(f"目标位置: {target:.4f} 圈")
    print(f"最大速度: {vel_max} 转/秒")
    print(f"加速度:   {acc} 转/秒^2")
    print(f"\n按 Enter 开始运动 (Ctrl+C 随时停止)...")

    try:
        input()
    except KeyboardInterrupt:
        print("\n已取消")
        ht.close()
        return

    print("开始运动...\n")
    try:
        # 发送梯形控制指令
        for i in range(50):
            s = ht.set_pos_vel_acc(MOTOR_ID, target, vel_max, acc)
            if s:
                print(f"  [{i+1:2d}] pos={s.position:.4f} 圈, vel={s.velocity:.4f} 转/秒, mode={s.mode}")
            else:
                print(f"  [{i+1:2d}] 未收到回复")
            time.sleep(0.1)

        print(f"\n运动完成，最终位置:")
        state = ht.read_motor_state(MOTOR_ID)
        if state:
            print(f"  位置: {state.position:.4f} 圈 (目标: {target:.4f})")

        # 等待 2 秒后返回原位
        print(f"\n2 秒后返回起始位置 ({start_pos:.4f} 圈)...")
        time.sleep(2)

        for i in range(50):
            s = ht.set_pos_vel_acc(MOTOR_ID, start_pos, vel_max, acc)
            if s:
                print(f"  [{i+1:2d}] pos={s.position:.4f} 圈, vel={s.velocity:.4f} 转/秒")
            time.sleep(0.1)

        print(f"\n回到起始位置:")
        state = ht.read_motor_state(MOTOR_ID)
        if state:
            print(f"  位置: {state.position:.4f} 圈")

    except KeyboardInterrupt:
        print("\n\n紧急停止！")

    # 停止电机
    print("\n发送停止指令...")
    ht.stop(MOTOR_ID)
    print("电机已停止")

    ht.close()
    print("\n=== 测试完成 ===")


if __name__ == "__main__":
    main()
