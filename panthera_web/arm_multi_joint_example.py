#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Python 侧多关节机械臂控制 — 与
Panthera-HT_SDK/panthera_cpp/motor_example_debug/src/11_multi_joint_control.cpp
对齐: S 曲线 + 一拖多 + run_control_loop + 完整交互菜单.

命令 (位置单位: 度, 与 C++ 一致):
  <ID> <位置>     单关节
  all <p1>..<pN>  全部关节同时到位 (N = len(motor_ids))
  home            全部回零
  brake / b       mode=0x0F
  free / stop     mode=0x00
  release / r     mode=0x0A
  stats           收发统计
  q               退出
运动中 ESC / Ctrl+Q: 紧急停止 (经 run_control_loop 的 abort_check).

用法:
  python arm_multi_joint_example.py [robot.cfg]
  python arm_multi_joint_example.py robot.cfg --demo-deg "1:2,3:0"   # 仅跑一次演示后退出

依赖: panthera_web 下已构建 panthera_motor.pyd 与 serial_cmake.dll.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Callable, Dict, List, Optional, Tuple

import panthera_motor as pm

PI = math.pi
VEL_AVG_MAX = 0.5
V_AVG_TARGET = 0.05
DT_MIN = 0.3
SETTLE_TICKS_MS = 300
DEG_PER_TURN = 360.0

MODE_POS = 0x0A
MODE_BRAKE = 0x0F
MODE_STOP = 0x00


class EmergencyStop(Exception):
    """运动中 / 行输入时 ESC 或 Ctrl+Q."""


def deg_to_turns(d: float) -> float:
    return d / DEG_PER_TURN


def turns_to_deg(t: float) -> float:
    return t * DEG_PER_TURN


def strip_lower(s: str) -> str:
    return " ".join(s.split()).lower()


def split_ws(s: str) -> List[str]:
    return s.split()


def starts_with(s: str, p: str) -> bool:
    return len(s) >= len(p) and s[: len(p)] == p


def parse_double(tok: str) -> Optional[float]:
    try:
        v = float(tok)
        if math.isnan(v) or math.isinf(v):
            return None
        return v
    except ValueError:
        return None


def parse_int(tok: str) -> Optional[int]:
    try:
        return int(tok, 10)
    except ValueError:
        return None


def poll_abort_key() -> bool:
    """非阻塞检测 ESC(0x1B) / Ctrl+Q(0x11), 供 run_control_loop.abort_check 使用."""
    if sys.platform == "win32":
        try:
            import msvcrt
        except ImportError:
            return False
        while msvcrt.kbhit():
            c = msvcrt.getch()
            ci = c[0] if isinstance(c, (bytes, bytearray)) else ord(c) if isinstance(c, str) else int(c)
            if ci in (0x1B, 0x11):
                return True
            if ci in (0x00, 0xE0) and msvcrt.kbhit():
                msvcrt.getch()
        return False

    import select

    if select.select([sys.stdin], [], [], 0)[0]:
        ch = sys.stdin.read(1)
        if not ch:
            return False
        o = ord(ch)
        return o in (0x1B, 0x11)
    return False


def input_line(prompt: str) -> str:
    """
    行输入. Windows 下用 msvcrt 实现与 C++ 类似的退格 / ESC / Ctrl+Q.
    其它平台退化为 input() (运动中仍可用 poll_abort_key).
    """
    print(prompt, end="", flush=True)
    if sys.platform != "win32":
        line = input()
        return strip_lower(line)

    import msvcrt

    buf: List[str] = []
    while True:
        c = msvcrt.getch()
        ci = c[0] if isinstance(c, (bytes, bytearray)) else ord(c)
        if ci in (0x1B, 0x11):
            print()
            raise EmergencyStop()
        if ci in (13, 10):
            print()
            time.sleep(0.002)
            while msvcrt.kbhit():
                nxt = msvcrt.getch()
                ni = nxt[0] if isinstance(nxt, (bytes, bytearray)) else ord(nxt)
                if ni not in (13, 10):
                    break
            return strip_lower("".join(buf))
        if ci in (8, 127):
            if buf:
                buf.pop()
                print("\b \b", end="", flush=True)
            continue
        if ci in (0x00, 0xE0):
            if msvcrt.kbhit():
                msvcrt.getch()
            continue
        ch = chr(ci) if 32 <= ci < 127 else ""
        if ch:
            buf.append(ch)
            print(ch, end="", flush=True)


def warn_if_out_of_limit(ht: pm.HightorqueSerial, motor_id: int, pos_deg: float) -> None:
    pos_turns = deg_to_turns(pos_deg)
    lim = ht.get_position_limit_turns(motor_id)
    if lim is None:
        return
    lo_t, hi_t = lim
    if lo_t <= pos_turns <= hi_t:
        return
    clamped = hi_t if pos_turns > hi_t else lo_t
    print(
        f"    ! 电机 {motor_id}: {pos_deg:.2f} 度 超出限位 "
        f"[{turns_to_deg(lo_t):.2f}, {turns_to_deg(hi_t):.2f}] 度, "
        f"将由驱动自动截断为 {turns_to_deg(clamped):.2f} 度"
    )


def switch_mode_all(
    ht: pm.HightorqueSerial,
    motor_ids: List[int],
    mode: int,
    label: str,
    max_retry: int = 3,
) -> bool:
    need_verify = mode in (MODE_STOP, MODE_BRAKE)
    print(f"  切换至 {label} (mode=0x{mode:02x})...")
    for attempt in range(1, max_retry + 1):
        failed: List[int] = []
        for mid in motor_ids:
            s = ht.set_motor_mode(mid, mode)
            if need_verify and s is None:
                time.sleep(0.02)
                s = ht.read_motor_state(mid, 0.15)
            got = s.mode if s is not None else -1
            if s is not None and got == mode:
                extra = f"  [重试 #{attempt}]" if attempt > 1 else ""
                print(f"    电机 {mid}: OK (mode=0x{got:02x}){extra}")
            else:
                failed.append(mid)
                fault = s.fault if s is not None else 0
                print(f"    电机 {mid}: 失败 (mode=0x{got:02x} fault=0x{fault:02x})")
        if not failed:
            return True
        if attempt < max_retry:
            print(f"    → 重试 {len(failed)} 个电机...")
            time.sleep(0.1)
    return False


def read_all_status(ht: pm.HightorqueSerial, motor_ids: List[int]) -> None:
    tag = " (async, 主动查询触发 cache 刷新)" if ht.is_async_rx() else ""
    print(f"\n  当前状态{tag}:")
    for mid in motor_ids:
        s = ht.read_motor_state(mid, 0.05)
        if s is None:
            print(f"    电机 {mid}: 50ms 内无回包 (检查 USB / 电机供电)")
            continue
        lim = ht.get_position_limit_turns(mid)
        if lim is not None:
            lo_t, hi_t = lim
            lim_s = f"限位=[{turns_to_deg(lo_t):+.1f}, {turns_to_deg(hi_t):+.1f}] 度"
        else:
            lim_s = "限位=(未设)"
        print(
            f"    电机 {mid}: pos={turns_to_deg(s.position):+.2f} 度  "
            f"vel={turns_to_deg(s.velocity):+.2f} 度/秒  mode={s.mode}  {lim_s}"
        )


def move_scurve(
    ht: pm.HightorqueSerial,
    motor_ids: List[int],
    targets_turns: Dict[int, float],
    cfg: pm.RobotConfig,
    abort_check: Optional[Callable[[], bool]] = None,
) -> None:
    """与 C++ move_all_async 等价的 S 曲线 + set_many + run_control_loop."""
    rate_hz = max(10.0, cfg.control_rate_hz)
    print(f"\n  运动中 ({int(rate_hz)}Hz 异步 + S 曲线, ESC/Ctrl+Q 紧急停止)...")

    if not ht.is_async_rx():
        print("  [警告] async RX 未启用, 临时降级为单帧同步发送 (会慢)")

    start_pos: Dict[int, float] = {}
    for mid in motor_ids:
        s = ht.read_motor_state(mid, 0.1)
        if s is None:
            print(f"  [错误] 拿不到电机 {mid} 的当前位置, 放弃运动以保安全")
            return
        start_pos[mid] = s.position

    dt_user = max(DT_MIN, cfg.trajectory_dt_s)
    max_abs_dpos = 0.0
    for mid in motor_ids:
        if mid not in targets_turns:
            continue
        max_abs_dpos = max(max_abs_dpos, abs(targets_turns[mid] - start_pos[mid]))

    dt_s = dt_user
    if max_abs_dpos > 1e-5:
        dt_target = max_abs_dpos / V_AVG_TARGET
        if dt_target < dt_user:
            dt_s = max(DT_MIN, dt_target)
            print(
                f"  [自适应 dt] 最大动作 {turns_to_deg(max_abs_dpos):.2f} 度, "
                f"段时长缩短为 {dt_s:.2f}s"
            )

    plans: Dict[int, Tuple[float, float]] = {}
    for mid in motor_ids:
        if mid not in targets_turns:
            continue
        dpos = targets_turns[mid] - start_pos[mid]
        if abs(dpos) < 1e-5:
            plans[mid] = (0.0, 0.0)
            continue
        v_avg = abs(dpos) / dt_s
        v_peak = min(VEL_AVG_MAX, v_avg) * (PI / 2.0)
        plans[mid] = (dpos, math.copysign(v_peak, dpos))

    total_ticks = max(1, int(dt_s * rate_hz))
    settle_ticks = max(1, int(SETTLE_TICKS_MS * rate_hz / 1000.0))
    last_tick = total_ticks + settle_ticks
    max_mid = max(motor_ids) if motor_ids else 0

    limit_reported = False
    print_every_tick = max(1, int(rate_hz * 0.4))
    last_print_tick = -print_every_tick

    def on_tick(tick: int, _dt_ms: float) -> bool:
        nonlocal limit_reported, last_print_tick
        if tick >= last_tick:
            return False

        alpha_raw = tick / total_ticks
        alpha = min(1.0, alpha_raw)
        smooth = 0.5 * (1.0 - math.cos(PI * alpha))
        vel_factor = math.sin(PI * alpha)

        cmds: List[pm.ManyMotorCmd] = []
        for mid in motor_ids:
            if mid in plans and plans[mid][1] != 0.0:
                dpos, v_peak_signed = plans[mid]
                desired = start_pos[mid] + smooth * dpos
                v_inst = v_peak_signed * vel_factor
                cmds.append(
                    pm.ManyMotorCmd(mid, desired, v_inst, cfg.max_torque_raw)
                )
            else:
                cmds.append(
                    pm.ManyMotorCmd(mid, start_pos[mid], 0.0, cfg.max_torque_raw)
                )

        states = ht.set_many_pos_vel_tqe(
            cmds, pm.PosUnit.Turns, max_mid, 0.002
        )
        if not limit_reported:
            for mid, st in states.items():
                if st.pos_limit_flag != 0:
                    print(
                        f"    [限位] 电机 {mid} 目标超界, 驱动已截断 "
                        f"(flag={st.pos_limit_flag})"
                    )
                    limit_reported = True
                    break

        if tick - last_print_tick >= print_every_tick or tick == 0:
            last_print_tick = tick
            cur = ht.get_states(motor_ids) if ht.is_async_rx() else states
            parts = []
            for mid in motor_ids:
                if mid not in targets_turns:
                    continue
                sit = cur.get(mid)
                if sit is None:
                    continue
                err_deg = turns_to_deg(targets_turns[mid] - sit.position)
                parts.append(
                    f"M{mid}={turns_to_deg(sit.position):+.2f}(err={err_deg:+.2f}°)"
                )
            stt = ht.get_stats()
            print(
                f"    [{tick + 1:4d}/{last_tick} α={alpha:.2f} v×={vel_factor:.2f}] "
                f"{' | '.join(parts)}  (jitter={stt.max_tx_jitter_ms:.2f}ms)"
            )
        return True

    ac = abort_check if abort_check is not None else poll_abort_key
    rc = ht.run_control_loop(
        rate_hz,
        motor_ids,
        on_tick,
        abort_check=ac,
        on_exception=lambda msg: print(f"  [异常] {msg}"),
        stop_on_finish=False,
        stop_on_abort=True,
    )
    if rc == 1:
        raise EmergencyStop()
    if rc == 2:
        print("  [警告] 控制环异常退出")
    print("  到位.", ht.get_stats().to_string())
    read_all_status(ht, motor_ids)


def pick_serial_port(preferred: str) -> str:
    """与 C++ pick_serial_port 等价: preferred 为空 / 'auto' 时枚举 USB 调试板自动选一个."""
    pref = (preferred or "").strip()
    is_auto = pref == "" or pref.lower() == "auto"

    try:
        usb_ports = pm.find_likely_debug_boards()
    except Exception as e:
        print(f"[端口] 枚举失败: {e}")
        return preferred

    print(f"[端口] 检测到 {len(usb_ports)} 个 USB 候选:")
    for p in usb_ports:
        star = "★ " if (not is_auto and p.port == pref) else "  "
        print(f"    {star}{p.port:<10}  {p.description}  [{p.hardware_id}]")

    if not is_auto:
        for p in usb_ports:
            if p.port == pref:
                return pref
        print(f"[端口] 偏好 {pref!r} 不在候选里, 回退自动选择")

    if not usb_ports:
        print("[端口] 未发现任何 USB 串口. 请检查调试板 USB 是否插好.")
        return preferred

    chosen = usb_ports[0].port
    if len(usb_ports) > 1:
        print(f"[端口] 多个候选, 默认选第一个: {chosen} (其它见上)")
    else:
        print(f"[端口] 自动选定: {chosen}")
    return chosen


def parse_demo_deg(s: str) -> Dict[int, float]:
    out: Dict[int, float] = {}
    for part in s.replace(",", " ").split():
        part = part.strip()
        if not part:
            continue
        sep = ":" if ":" in part else ("=" if "=" in part else None)
        if sep is None:
            raise ValueError(f"无法解析片段: {part!r} (需要 id:度 或 id=度)")
        a, b = part.split(sep, 1)
        mid = int(a.strip())
        deg = float(b.strip())
        out[mid] = deg_to_turns(deg)
    return out


def interactive_loop(ht: pm.HightorqueSerial, cfg: pm.RobotConfig) -> None:
    motor_ids = cfg.motor_ids
    is_disabled = False

    def ensure_position_mode() -> bool:
        nonlocal is_disabled
        if not is_disabled:
            return True
        print("  [自动] 当前处于 brake/free 状态, 切回位置控制...")
        if switch_mode_all(ht, motor_ids, MODE_POS, "位置控制", 2):
            is_disabled = False
            time.sleep(0.05)
            return True
        print("  [失败] 无法切回位置控制, 取消运动")
        return False

    n = len(motor_ids)
    while True:
        read_all_status(ht, motor_ids)

        print(
            f"\n  命令 (位置单位: 度):\n"
            f"    <ID> <位置度>      单关节运动 (例: 1 36.0)\n"
            f"    all <p1>..<p{n}>     {n} 关节同时设置 (单位: 度)\n"
            f"    home               所有关节回零\n"
            f"    brake / b          全部刹车 (mode=0x0F)\n"
            f"    free  / stop       全部断电 (mode=0x00)\n"
            f"    release / r        切回位置控制 (mode=0x0A)\n"
            f"    stats              查看 TX/RX 统计\n"
            f"    q                  退出\n"
            f"    ESC / Ctrl+Q       紧急停止 (运动中 / 本行输入中, Windows 控制台)\n"
        )
        if is_disabled:
            print("  ⚠️ 当前: brake/free 状态, 下次运动会自动 release")

        try:
            cmd = input_line("\n  > ")
        except EmergencyStop:
            print("\n\n紧急停止 (输入行 ESC/Ctrl+Q)!\n")
            continue

        if not cmd:
            continue
        if cmd == "q":
            break

        if cmd == "stats":
            print(" ", ht.get_stats().to_string())
            continue

        if cmd in ("brake", "b"):
            if switch_mode_all(ht, motor_ids, MODE_BRAKE, "刹车 (短路阻尼)", 1):
                is_disabled = True
                print("  → 全部电机已刹车. 下次运动命令会自动 release.")
            continue
        if cmd in ("free", "stop"):
            if switch_mode_all(ht, motor_ids, MODE_STOP, "停止 (无电流, 可手动转)", 1):
                is_disabled = True
                print("  → 全部电机已断电. 下次运动命令会自动 release.")
            continue
        if cmd in ("release", "r"):
            if switch_mode_all(ht, motor_ids, MODE_POS, "位置控制 hold", 2):
                is_disabled = False
                print("  → 全部电机已 release, 当前位置 hold 中.")
            continue

        if cmd == "home":
            targets: Dict[int, float] = {mid: 0.0 for mid in motor_ids}
            for mid in motor_ids:
                warn_if_out_of_limit(ht, mid, 0.0)
            print("  目标: 全部回零")
            try:
                confirm = input_line("  确认? (Enter=执行, q=取消): ")
            except EmergencyStop:
                print("\n\n紧急停止!\n")
                continue
            if confirm == "q":
                continue
            if not ensure_position_mode():
                continue
            try:
                move_scurve(ht, motor_ids, targets, cfg)
            except EmergencyStop:
                print("\n\n紧急停止 (ESC / Ctrl+Q)!\n")
            continue

        if starts_with(cmd, "all"):
            parts = split_ws(cmd)
            if len(parts) != 1 + n:
                print(f"  格式错误, 需要 {n} 个位置值 (单位: 度)")
                continue
            targets: Dict[int, float] = {}
            ok = True
            for i in range(n):
                pos_deg = parse_double(parts[1 + i])
                if pos_deg is None:
                    ok = False
                    break
                mid = motor_ids[i]
                warn_if_out_of_limit(ht, mid, pos_deg)
                targets[mid] = deg_to_turns(pos_deg)
            if not ok:
                print("  位置值格式错误, 请输入数字 (单位: 度)")
                continue
            line = ", ".join(
                f"M{mid}={turns_to_deg(targets[mid]):+.2f}°" for mid in motor_ids
            )
            print(f"  目标: {line}")
            try:
                confirm = input_line("  确认? (Enter=执行, q=取消): ")
            except EmergencyStop:
                print("\n\n紧急停止!\n")
                continue
            if confirm == "q":
                continue
            if not ensure_position_mode():
                continue
            try:
                move_scurve(ht, motor_ids, targets, cfg)
            except EmergencyStop:
                print("\n\n紧急停止 (ESC / Ctrl+Q)!\n")
            continue

        parts = split_ws(cmd)
        if len(parts) == 2:
            mid = parse_int(parts[0])
            pos_deg = parse_double(parts[1])
            if mid is None or pos_deg is None:
                print("  格式错误, 请输入: <电机ID> <位置(度)>")
                continue
            if mid not in motor_ids:
                print(f"  电机 {mid} 不在配置的电机列表中")
                continue
            warn_if_out_of_limit(ht, mid, pos_deg)
            pos_turns = deg_to_turns(pos_deg)
            print(f"  目标: 电机 {mid} -> {pos_deg:+.2f} 度")
            try:
                confirm = input_line("  确认? (Enter=执行, q=取消): ")
            except EmergencyStop:
                print("\n\n紧急停止!\n")
                continue
            if confirm == "q":
                continue
            if not ensure_position_mode():
                continue
            try:
                move_scurve(ht, motor_ids, {mid: pos_turns}, cfg)
            except EmergencyStop:
                print("\n\n紧急停止 (ESC / Ctrl+Q)!\n")
            continue

        print("  未识别命令")


def run_demo_then_exit(ht: pm.HightorqueSerial, cfg: pm.RobotConfig, spec: str) -> int:
    targets = parse_demo_deg(spec)
    for mid in targets:
        if mid not in cfg.motor_ids:
            print(f"[错误] 电机 {mid} 不在配置 motor_ids 中")
            return 1
    print("\n  演示目标 (度):", {k: turns_to_deg(v) for k, v in targets.items()})
    try:
        move_scurve(ht, cfg.motor_ids, targets, cfg)
    except EmergencyStop:
        print("\n\n紧急停止 (ESC / Ctrl+Q)!\n")
        return 1
    return 0


def main() -> int:
    import os

    script_dir = os.path.dirname(os.path.abspath(__file__))
    default_cfg = os.path.join(script_dir, "robot.cfg")

    ap = argparse.ArgumentParser(description="Panthera 机械臂 Python 控制台 (pybind panthera_motor)")
    ap.add_argument(
        "config",
        nargs="?",
        default=default_cfg,
        help=f"robot.cfg 路径 (默认: {default_cfg})",
    )
    ap.add_argument(
        "--demo-deg",
        metavar="SPEC",
        help='仅跑一次演示后退出, 如 "1:10,3=-20" (度)',
    )
    args = ap.parse_args()

    cfg_path = args.config
    if not os.path.isabs(cfg_path) and not os.path.exists(cfg_path):
        candidate = os.path.join(script_dir, cfg_path)
        if os.path.exists(candidate):
            cfg_path = candidate

    try:
        cfg = pm.RobotConfig.load(cfg_path)
    except RuntimeError as e:
        print(f"[配置] 加载失败: {e}")
        return 1
    print(f"[配置] 已加载: {cfg_path}")

    if not cfg.motor_ids:
        print("[错误] motor_ids 为空")
        return 1

    print(cfg)

    cfg.port = pick_serial_port(cfg.port)

    try:
        ht = pm.HightorqueSerial(cfg.port, cfg.baudrate)
    except RuntimeError as e:
        print(f"[错误] 无法打开串口 {cfg.port!r}: {e}")
        return 1

    exit_code = 0
    try:
        cfg.apply_limits_to(ht)

        print("\n--- 通信预检 ---")
        all_ok = True
        for mid in cfg.motor_ids:
            s = ht.read_motor_state(mid)
            if s is None:
                print(f"  电机 {mid}: 无响应!")
                all_ok = False
            else:
                print(f"  电机 {mid}: OK (pos={turns_to_deg(s.position):+.2f}°)")
        if not all_ok:
            return 1

        print("\n--- 切到位置模式 (mode=0x0A), 须在 async 之前 ---")
        if not switch_mode_all(ht, cfg.motor_ids, MODE_POS, "位置控制", max_retry=3):
            print("[失败] 无法切到位置模式, 请断电重启电机后重试")
            return 1

        if cfg.use_async_rx:
            print("\n[异步 RX] 启动...")
            ht.enable_async_rx()
            time.sleep(0.1)
            for mid in cfg.motor_ids:
                ht.read_motor_state(mid, 0.1)
            cached = sum(1 for mid in cfg.motor_ids if ht.get_state(mid) is not None)
            print(f"[异步 RX] cache {cached}/{len(cfg.motor_ids)}  ", ht.get_stats().to_string())

        read_all_status(ht, cfg.motor_ids)

        if args.demo_deg:
            exit_code = run_demo_then_exit(ht, cfg, args.demo_deg)
        else:
            try:
                interactive_loop(ht, cfg)
            except EmergencyStop:
                print("\n\n紧急停止!\n")
                exit_code = 1

    finally:
        if ht.is_async_rx():
            ht.disable_async_rx()
        for mid in cfg.motor_ids:
            try:
                ht.stop(mid)
            except RuntimeError:
                pass
        ht.close()

    print(f"\n程序结束 (exit={exit_code})")
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
