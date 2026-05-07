#!/usr/bin/env python3
"""
测试脚本：通过调试板与电机通信，读取状态。
"""
import sys
import time
from hightorque_serial import HightorqueSerial

PORT = "COM14"
BAUDRATE = 4000000


def main():
    print(f"=== 连接 {PORT} (波特率 {BAUDRATE}) ===\n")
    ht = HightorqueSerial(PORT, BAUDRATE)

    # 步骤 1: 检查 CAN 状态
    print("[步骤 1] CAN 总线状态")
    print(f"  状态: {ht.can_status()}")
    print(f"  配置: {ht.can_config()}")

    # 步骤 2: 读取电机 1 状态
    print("\n[步骤 2] 读取电机 1 状态 (10 次)...")
    success = 0
    for i in range(10):
        state = ht.read_motor_state(1, timeout=0.5)
        if state:
            success += 1
            print(f"  [{i+1}] {state}")
        else:
            print(f"  [{i+1}] 未收到回复")
        time.sleep(0.1)
    print(f"  成功率: {success}/10")

    # 步骤 3: 读取电机 7 状态
    print("\n[步骤 3] 读取电机 7 状态...")
    state7 = ht.read_motor_state(7, timeout=0.5)
    if state7:
        print(f"  电机 7: {state7}")
    else:
        print("  电机 7 未回复")

    # 步骤 4: 查询电机固件版本
    print("\n[步骤 4] 查询电机固件版本...")
    ver1 = ht.read_version(1)
    print(f"  电机 1: {ver1 or '未回复'}")
    ver7 = ht.read_version(7)
    print(f"  电机 7: {ver7 or '未回复'}")

    # 步骤 5: 发送停止指令
    print("\n[步骤 5] 发送停止指令...")
    s1 = ht.stop(1)
    print(f"  电机 1: {s1 or '未回复'}")
    s7 = ht.stop(7)
    print(f"  电机 7: {s7 or '未回复'}")

    ht.close()
    print("\n=== 测试完成 ===")


if __name__ == "__main__":
    main()
