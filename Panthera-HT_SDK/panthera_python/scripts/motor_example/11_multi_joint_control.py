#!/usr/bin/env python3
"""
11 - 多关节交互式位置控制 (七自由度机械臂)

交互式输入各关节目标位置 (圈), 使用梯形控制安全运动。
内置软限位保护, Ctrl+Q 随时紧急停止。

⚠ 机械臂场景安全提醒:
  - 首次运行先用 10_test_single_joint.py 确认各关节正常
  - 肩关节(2号)和肘关节(3号)承重, stop 后保持力矩
  - 4～7 号限位为保守初值, 务必按实际行程在 JOINT_LIMITS 中修改
  - 如电机失控, 直接拔掉调试板 USB 线
"""
import os
import sys
import time
from hightorque_serial import HightorqueSerial


class EmergencyStop(Exception):
    """由 Ctrl+Q 触发的紧急停止。"""


def _poll_ctrl_q() -> bool:
    """非阻塞检测是否按下 Ctrl+Q (ASCII 0x11)。"""
    if sys.platform == "win32":
        import msvcrt

        while msvcrt.kbhit():
            ch = msvcrt.getch()
            if ch == b"\x11":
                return True
            if ch in (b"\xe0", b"\x00") and msvcrt.kbhit():
                msvcrt.getch()
        return False
    import select

    if not hasattr(select, "select"):
        return False
    try:
        r, _, _ = select.select([sys.stdin], [], [], 0)
    except (ValueError, OSError):
        return False
    if not r:
        return False
    try:
        ch = os.read(sys.stdin.fileno(), 1)
    except OSError:
        return False
    return ch == b"\x11"


def _sleep_abortable(seconds: float) -> None:
    end = time.monotonic() + seconds
    while True:
        if _poll_ctrl_q():
            raise EmergencyStop()
        now = time.monotonic()
        if now >= end:
            return
        time.sleep(min(0.05, end - now))


def input_line(prompt: str) -> str:
    """读取一行; 任意时刻 Ctrl+Q 触发 EmergencyStop。"""
    sys.stdout.write(prompt)
    sys.stdout.flush()
    if sys.platform == "win32":
        import msvcrt

        buf = bytearray()
        while True:
            ch = msvcrt.getch()
            if ch == b"\x11":
                raise EmergencyStop()
            if ch in (b"\r", b"\n"):
                sys.stdout.write("\n")
                sys.stdout.flush()
                return buf.decode("utf-8", errors="replace").strip()
            if ch in (b"\x08", b"\x7f"):
                if buf:
                    buf.pop()
                    sys.stdout.write("\b \b")
                    sys.stdout.flush()
            elif ch in (b"\xe0", b"\x00") and msvcrt.kbhit():
                msvcrt.getch()
            else:
                buf.extend(ch)
                try:
                    sys.stdout.write(ch.decode("utf-8", errors="replace"))
                except UnicodeDecodeError:
                    pass
                sys.stdout.flush()
    import termios
    import tty

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        buf = bytearray()
        while True:
            ch = os.read(fd, 1)
            if not ch:
                continue
            if ch == b"\x11":
                raise EmergencyStop()
            if ch in (b"\r", b"\n"):
                sys.stdout.write("\r\n")
                sys.stdout.flush()
                return buf.decode("utf-8", errors="replace").strip()
            if ch in (b"\x7f", b"\x08"):
                if buf:
                    buf.pop()
                    sys.stdout.write("\b \b")
                    sys.stdout.flush()
            elif ch[0] >= 32:
                buf.extend(ch)
                sys.stdout.write(ch.decode("utf-8", errors="replace"))
                sys.stdout.flush()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

PORT = "COM14"
BAUDRATE = 4_000_000
MOTOR_IDS = [1, 2, 3, 4, 5, 6, 7]

VEL_MAX = 0.05       # 最大速度 (转/秒)
ACC = 0.05            # 加速度 (转/秒²)
MOVE_STEPS = 40       # 运动步数 (每步 0.1s → 4s)
MOVE_DT = 0.1

# 各关节软限位 (圈): {motor_id: (min, max)}
# 根据实际机械臂结构调整这些值!
JOINT_LIMITS = {
    1: (-0.40, 0.30),   # 底座
    2: (-0.05, 0.48),   # 肩
    3: (0.0, 0.47),     # 肘
    # 以下为 4～7 保守初值 (±0.25 圈 ≈ ±90°), 请按机械限位实测后修改
    4: (-0.25, 0.25),
    5: (-0.25, 0.25),
    6: (-0.25, 0.25),
    7: (-0.25, 0.25),   # 末端关节 (总线末端电阻仅影响通信, 与限位无关)
}


def clamp_position(motor_id: int, pos: float) -> float:
    lo, hi = JOINT_LIMITS.get(motor_id, (-1.0, 1.0))
    clamped = max(lo, min(hi, pos))
    if clamped != pos:
        print(f"    ! 电机 {motor_id}: {pos:.4f} 超出限位 [{lo}, {hi}], 截断为 {clamped:.4f}")
    return clamped


def read_all(ht: HightorqueSerial):
    """读取并打印所有关节状态。"""
    print("\n  当前状态:")
    for mid in MOTOR_IDS:
        s = ht.read_motor_state(mid)
        if s:
            lo, hi = JOINT_LIMITS.get(mid, (-1.0, 1.0))
            print(
                f"    电机 {mid}: pos={s.position:+.4f} 圈  "
                f"vel={s.velocity:+.4f}  mode={s.mode}  "
                f"限位=[{lo:+.2f}, {hi:+.2f}]"
            )
        else:
            print(f"    电机 {mid}: 无响应")


def move_all(ht: HightorqueSerial, targets: dict[int, float]):
    """同时控制所有关节到目标位置。"""
    print("\n  运动中... (Ctrl+Q 紧急停止)")
    for step in range(MOVE_STEPS):
        if _poll_ctrl_q():
            raise EmergencyStop()
        states = {}
        for mid in MOTOR_IDS:
            if mid in targets:
                s = ht.set_pos_vel_acc(mid, targets[mid], VEL_MAX, ACC)
                if s:
                    states[mid] = s

        if step % 5 == 0:
            parts = []
            for mid in MOTOR_IDS:
                if mid in states:
                    s = states[mid]
                    err = targets[mid] - s.position
                    parts.append(f"M{mid}={s.position:+.4f}(err={err:+.4f})")
            print(f"    [{step + 1:2d}/{MOVE_STEPS}] {' | '.join(parts)}")

        _sleep_abortable(MOVE_DT)

    # 到位后多发几次指令让电机稳定
    for _ in range(5):
        if _poll_ctrl_q():
            raise EmergencyStop()
        for mid in MOTOR_IDS:
            if mid in targets:
                ht.set_pos_vel_acc(mid, targets[mid], VEL_MAX, ACC)
        _sleep_abortable(0.05)

    print("  到位。")
    read_all(ht)


def interactive_loop(ht: HightorqueSerial):
    """交互式控制主循环。"""
    while True:
        read_all(ht)

        print("\n  命令:")
        print("    输入目标位置, 格式: <电机ID> <位置(圈)>   例如: 1 0.1")
        print("    输入 'all <p1>..<p7>'  同时设置七个关节 (顺序对应 ID 1～7)")
        print("    输入 'home'  所有关节回零")
        print("    输入 'q'  退出")
        print("    Ctrl+Q  紧急停止 (运动中与输入时均有效)")

        cmd = input_line("\n  > ").strip().lower()

        if cmd == "q":
            break

        if cmd == "home":
            targets = {mid: clamp_position(mid, 0.0) for mid in MOTOR_IDS}
            print(f"  目标: 全部回零")
            confirm = input_line("  确认? (Enter=执行, q=取消): ").strip().lower()
            if confirm == "q":
                continue
            move_all(ht, targets)
            continue

        if cmd.startswith("all"):
            parts = cmd.split()
            if len(parts) != 1 + len(MOTOR_IDS):
                print(f"  格式错误, 需要 {len(MOTOR_IDS)} 个位置值")
                continue
            try:
                targets = {}
                for i, mid in enumerate(MOTOR_IDS):
                    pos = float(parts[1 + i])
                    targets[mid] = clamp_position(mid, pos)
                print("  目标: " + ", ".join(f"M{mid}={targets[mid]:+.4f}" for mid in MOTOR_IDS))
                confirm = input_line("  确认? (Enter=执行, q=取消): ").strip().lower()
                if confirm == "q":
                    continue
                move_all(ht, targets)
            except ValueError:
                print("  位置值格式错误, 请输入数字")
            continue

        # 单关节控制: "<id> <pos>"
        parts = cmd.split()
        if len(parts) == 2:
            try:
                mid = int(parts[0])
                pos = float(parts[1])
                if mid not in MOTOR_IDS:
                    print(f"  电机 {mid} 不在列表 {MOTOR_IDS} 中")
                    continue
                pos = clamp_position(mid, pos)
                print(f"  目标: 电机 {mid} → {pos:+.4f} 圈")
                confirm = input_line("  确认? (Enter=执行, q=取消): ").strip().lower()
                if confirm == "q":
                    continue
                move_all(ht, {mid: pos})
            except ValueError:
                print("  格式错误, 请输入: <电机ID> <位置(圈)>")
            continue

        print("  未识别命令")


if __name__ == "__main__":
    ht = HightorqueSerial(PORT, BAUDRATE)

    print("=" * 56)
    print("  多关节交互式控制")
    print(f"  电机: {MOTOR_IDS}")
    print(f"  速度: {VEL_MAX} 转/秒, 加速度: {ACC} 转/秒²")
    print("  限位:")
    for mid in MOTOR_IDS:
        lo, hi = JOINT_LIMITS.get(mid, (-1.0, 1.0))
        print(f"    电机 {mid}: [{lo:+.2f}, {hi:+.2f}] 圈 ({lo * 360:+.0f}° ~ {hi * 360:+.0f}°)")
    print("=" * 56)

    # 通信检查
    print("\n--- 通信检查 ---")
    all_ok = True
    for mid in MOTOR_IDS:
        state = ht.read_motor_state(mid)
        if state:
            print(f"  电机 {mid}: OK (pos={state.position:+.4f})")
        else:
            print(f"  电机 {mid}: 无响应!")
            all_ok = False

    if not all_ok:
        print("\n有电机无响应，请检查连接。")
        ht.close()
        exit(1)

    try:
        interactive_loop(ht)
    except EmergencyStop:
        print("\n\n紧急停止 (Ctrl+Q)!")
    except KeyboardInterrupt:
        print("\n\n紧急停止 (Ctrl+C)!")

    for mid in MOTOR_IDS:
        ht.stop(mid)
    ht.close()
    print("\n程序结束")
