# -*- coding: utf-8 -*-
"""
test_fafu_motion_interactive.py
===============================

带菜单的交互式实验脚本 (基于 fafu_robot_controller 的高层封装).
不用反复改代码就能跑遍 FafuRobotController 的各种功能
(单关节 / 多关节 / 夹爪 / 软限位 / 状态监视 / 示教复现).

⚠️ 第一次跑前:
    - 机器臂上电
    - 桌面留出空间, 手放在键盘 Ctrl+C 上, 急停就在旁边
    - 推荐 --speed 不超过 15

用法:
    cd fafu_robot_sdk
    python tests/test_fafu_motion_interactive.py                      # 默认无夹爪
    python tests/test_fafu_motion_interactive.py --gripper-id 7       # 7 号是夹爪
    python tests/test_fafu_motion_interactive.py --speed 10           # 全局降速
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time
import traceback
from typing import Callable, Dict, List, Optional

import numpy as np


HERE = os.path.dirname(os.path.abspath(__file__))
PARENT = os.path.dirname(HERE)
if PARENT not in sys.path:
    sys.path.insert(0, PARENT)


# ============================================================================
#  Helpers
# ============================================================================
def deg_str(angles_rad) -> str:
    return "[" + ", ".join(f"{math.degrees(v):+7.2f}" for v in angles_rad) + "]"


def parse_floats(s: str, sep: Optional[str] = None) -> List[float]:
    """'10 20 30' / '10,20,30' / '10, 20, 30' -> [10.0, 20.0, 30.0]."""
    s = s.replace(",", " ").strip()
    out: List[float] = []
    for tok in s.split():
        try:
            out.append(float(tok))
        except ValueError:
            raise ValueError(f"无法解析为数字: {tok!r}")
    return out


def prompt(text: str) -> str:
    try:
        return input(text).strip()
    except EOFError:
        return ""


def yes(s: str) -> bool:
    return s.lower() in ("y", "yes", "ok", "ok!", "")


# ============================================================================
#  Menu items
# ============================================================================
class App:
    def __init__(self, arm, default_speed: int):
        from fafu_robot_controller import FafuRobotController
        assert isinstance(arm, FafuRobotController)
        self.arm = arm
        self.default_speed = default_speed
        # 示教录制的路点 (rad list 列表)
        self.taught_path: List[List[float]] = []
        # 自定义书签: name -> (joint_angles_rad, gripper_angle_rad_or_None)
        self.bookmarks: Dict[str, tuple] = {}

    # ----- 状态/信息 -----

    def show_state(self):
        q = self.arm.get_joint_values()
        print(f"\n  joint motor ids: {self.arm.joint_motor_ids}")
        print(f"  joint angles   : {deg_str(q)} (deg)")
        try:
            qd = self.arm.get_joint_velocities()
            print(f"  joint vel      : {deg_str(qd)} (deg/s)")
        except Exception as e:
            print(f"  joint vel      : <读取失败: {e}>")

        if self.arm.has_gripper:
            try:
                gs = self.arm.get_gripper_state()
                ang_deg = math.degrees(self.arm._turns_to_rad(gs.position))
                print(f"  gripper M{self.arm.gripper_motor_id:<2}    : {ang_deg:+.2f} deg "
                      f"(mode=0x{gs.mode:02X}, fault=0x{gs.fault:02X})")
            except Exception as e:
                print(f"  gripper        : <读取失败: {e}>")

        print(f"  is_enabled     : {self.arm.is_enabled}")
        print(f"  stats          : {self.arm.get_status().to_string()}")

    def show_limits(self):
        print()
        for mid in self.arm.all_motor_ids:
            lim = self.arm.get_limit(mid)
            tag = " (gripper)" if (self.arm.has_gripper
                                   and mid == self.arm.gripper_motor_id) else ""
            if lim is None:
                print(f"  M{mid}{tag}: 未设软限位")
            else:
                lo, hi = lim
                print(f"  M{mid}{tag}: [{math.degrees(lo):+8.2f}, "
                      f"{math.degrees(hi):+8.2f}] deg")

    # ----- 单关节运动 -----

    def move_one_joint(self):
        n = self.arm.num_joints
        s = prompt(f"  动哪个关节 (0..{n - 1}, 默认 {n - 1}): ")
        try:
            idx = int(s) if s else (n - 1)
        except ValueError:
            print("  非法编号"); return
        if not (0 <= idx < n):
            print(f"  超出范围 [0, {n})"); return

        s = prompt("  目标角度 (度, 例 +30): ")
        try:
            target_deg = float(s)
        except ValueError:
            print("  非法数字"); return

        s = prompt(f"  speed (默认 {self.default_speed}): ")
        speed = int(s) if s else self.default_speed

        q0 = self.arm.get_joint_values()
        target = q0.copy()
        target[idx] = math.radians(target_deg)
        print(f"\n  当前 (deg): {deg_str(q0)}")
        print(f"  目标 (deg): {deg_str(target)}")
        if not yes(prompt("  执行? [Y/n]: ")):
            print("  取消."); return
        try:
            self.arm.move_j(target, speed=speed, block=True)
        except Exception as e:
            traceback.print_exc()
            print(f"  [失败] {e}"); return
        time.sleep(0.2)
        q1 = self.arm.get_joint_values()
        err = math.degrees(q1[idx] - target[idx])
        print(f"  到位 (deg): {deg_str(q1)}  关节 {idx} 误差 {err:+.3f}°")

    # ----- 多关节运动 -----

    def move_all_joints(self):
        n = self.arm.num_joints
        print(f"  输入 {n} 个角度 (度), 空格或逗号分隔:")
        s = prompt(f"  > ")
        try:
            vals = parse_floats(s)
        except ValueError as e:
            print(f"  {e}"); return
        if len(vals) != n:
            print(f"  需要 {n} 个值, 给了 {len(vals)} 个"); return

        s = prompt(f"  speed (默认 {self.default_speed}): ")
        speed = int(s) if s else self.default_speed

        target = np.radians(vals)
        q0 = self.arm.get_joint_values()
        print(f"\n  当前 (deg): {deg_str(q0)}")
        print(f"  目标 (deg): {deg_str(target)}")
        if not yes(prompt("  执行? [Y/n]: ")):
            print("  取消."); return
        try:
            self.arm.move_j(target, speed=speed, block=True)
        except Exception as e:
            traceback.print_exc()
            print(f"  [失败] {e}"); return
        time.sleep(0.2)
        print(f"  到位 (deg): {deg_str(self.arm.get_joint_values())}")

    def go_home(self):
        s = prompt(f"  speed (默认 {self.default_speed}): ")
        speed = int(s) if s else self.default_speed
        if not yes(prompt("  全部关节回零? [Y/n]: ")):
            print("  取消."); return
        try:
            self.arm.go_home(speed=speed, block=True)
        except Exception as e:
            traceback.print_exc()
            print(f"  [失败] {e}"); return
        print(f"  到位 (deg): {deg_str(self.arm.get_joint_values())}")

    # ----- 夹爪 -----

    def gripper_menu(self):
        if not self.arm.has_gripper:
            print("\n  没有配置夹爪 (启动时未传 --gripper-id)")
            return
        while True:
            try:
                gs = self.arm.get_gripper_state()
                cur_deg = math.degrees(self.arm._turns_to_rad(gs.position))
                print(f"\n  当前夹爪角度: {cur_deg:+.2f} deg")
            except Exception as e:
                print(f"\n  读取失败: {e}")
            print("    [o] open  (软限位上限, 阻塞)")
            print("    [c] close (软限位下限, 阻塞)")
            print("    [a] 自定义角度")
            print("    [v] 设速度 / 当前默认 vel=0.3 turns/s")
            print("    [b] 返回主菜单")
            cmd = prompt("  > ").lower()
            if cmd in ("b", "back", "q", "exit", ""):
                return
            elif cmd == "o":
                self.arm.open_gripper()
            elif cmd == "c":
                self.arm.close_gripper()
            elif cmd == "a":
                s = prompt("  目标角度 (度): ")
                try:
                    deg = float(s)
                except ValueError:
                    print("  非法数字"); continue
                self.arm.gripper_control(angle=math.radians(deg))
            elif cmd == "v":
                print("  (跳过, 留作 TODO)")
            else:
                print(f"  未识别: {cmd!r}")

    # ----- 软限位 -----

    def limit_menu(self):
        while True:
            print("\n  软限位:")
            self.show_limits()
            print("\n    [s] set   <id> <lo_deg> <hi_deg>")
            print("    [d] disable <id>")
            print("    [c] clear all")
            print("    [b] 返回")
            cmd = prompt("  > ").lower()
            if cmd in ("b", "back", "q", "exit", ""):
                return
            if cmd == "c":
                if yes(prompt("  确认清空全部? [Y/n]: ")):
                    self.arm.clear_limits()
                    print("  已清空")
                continue
            tokens = cmd.split()
            if len(tokens) >= 2 and tokens[0] == "d":
                try:
                    mid = int(tokens[1])
                    self.arm.disable_limit(mid)
                    print(f"  已禁用 M{mid}")
                except Exception as e:
                    print(f"  失败: {e}")
                continue
            if len(tokens) == 4 and tokens[0] == "s":
                try:
                    mid = int(tokens[1])
                    lo = float(tokens[2])
                    hi = float(tokens[3])
                    self.arm.set_limit(mid, lo=lo, hi=hi, is_radians=False)
                    print(f"  已设 M{mid}: [{lo:+.2f}, {hi:+.2f}] deg")
                except Exception as e:
                    print(f"  失败: {e}")
                continue
            print(f"  未识别: {cmd!r}")

    # ----- 状态实时监视 (Ctrl+C 退出) -----

    def monitor(self, hz: float = 4.0):
        print("\n  [实时监视] Ctrl+C 退出")
        period = 1.0 / max(0.5, hz)
        try:
            while True:
                q = self.arm.get_joint_values()
                line = "  " + deg_str(q)
                if self.arm.has_gripper:
                    try:
                        gs = self.arm.get_gripper_state()
                        line += f"  | grip {math.degrees(self.arm._turns_to_rad(gs.position)):+6.1f}°"
                    except Exception:
                        pass
                print(line, flush=True)
                time.sleep(period)
        except KeyboardInterrupt:
            print("\n  [退出监视]")

    # ----- 示教 / 复现 -----

    def teach_record(self):
        print("\n  示教录制:")
        print("    1) 自动 disable 所有关节, 你用手拖动到目标位姿")
        print("    2) 每按一次 Enter 就记录一个路点")
        print("    3) 输入 'q' Enter 结束并自动 enable")
        if not yes(prompt("\n  开始? [Y/n]: ")):
            return
        try:
            self.arm.disable()
        except Exception as e:
            print(f"  disable 失败: {e}"); return
        recorded: List[List[float]] = []
        try:
            while True:
                cmd = prompt(f"  按 Enter 记录路点 #{len(recorded) + 1}, 'q' 结束: ")
                if cmd.lower() in ("q", "quit", "exit"):
                    break
                q = self.arm.get_joint_values(prefer_cache=False)
                recorded.append(q.tolist())
                print(f"     ✓ #{len(recorded)}: {deg_str(q)}")
        finally:
            print("\n  恢复 enable...")
            try:
                self.arm.enable()
            except Exception as e:
                print(f"  enable 失败: {e}")
        if recorded:
            self.taught_path = recorded
            print(f"  已保存 {len(recorded)} 个路点 (内存中)")
        else:
            print("  未记录任何路点")

    def teach_replay(self):
        if not self.taught_path:
            print("\n  未录制任何路点, 先用 [t] 录制"); return
        s = prompt(f"  speed (默认 {self.default_speed}): ")
        speed = int(s) if s else self.default_speed
        n = len(self.taught_path)
        print(f"\n  复现 {n} 个路点 (speed={speed}, 阻塞 block=True)")
        if not yes(prompt("  开始? [Y/n]: ")):
            return
        try:
            for i, q in enumerate(self.taught_path, start=1):
                print(f"  [{i}/{n}] -> {deg_str(q)}")
                self.arm.move_j(q, speed=speed, block=True)
                time.sleep(0.2)
            print("  完成")
        except KeyboardInterrupt:
            print("\n  [中断] 已停止当前运动"); self.arm.emergency_stop()
        except Exception as e:
            traceback.print_exc(); print(f"  [失败] {e}")

    # ----- 安全 -----

    def estop(self):
        if yes(prompt("\n  确认急停 (mode=0x00)? [Y/n]: ")):
            self.arm.emergency_stop()
            print("  急停已发. 用 'r' 恢复.")

    def resume(self):
        if yes(prompt("\n  确认恢复 (重新 enable)? [Y/n]: ")):
            try:
                self.arm.resume()
            except Exception as e:
                print(f"  失败: {e}")


# ============================================================================
#  Main loop
# ============================================================================
MENU = """
+--------------------------------------------------+
|  FafuRobotController 交互式测试                  |
+--------------------------------------------------+
  [s]  显示当前状态 (角度 / 速度 / 夹爪 / 统计)
  [j]  单关节移动 (选编号 + 目标角度)
  [a]  全部关节移动 (输入 N 个角度)
  [h]  回零 (go_home)
  [g]  夹爪子菜单
  [l]  软限位子菜单
  [t]  示教录制 (手动拖动 + 按 Enter 记录)
  [p]  复现已录制的路点
  [m]  实时状态监视 (Ctrl+C 退出)
  [e]  急停
  [r]  从急停恢复 (enable)
  [q]  退出
"""


def main() -> int:
    parser = argparse.ArgumentParser(description="interactive test menu")
    parser.add_argument("--cfg", default="robot.cfg",
                        help="robot.cfg 路径 (默认: fafu_robot_sdk/robot.cfg)")
    parser.add_argument("--gripper-id", type=int, default=None,
                        help="夹爪 motor id, 不传则视为无夹爪")
    parser.add_argument("--speed", type=int, default=15,
                        help="默认 speed percent (默认 15)")
    args = parser.parse_args()

    try:
        from fafu_robot_controller import FafuRobotController
    except Exception as e:
        traceback.print_exc()
        print(f"\n[FAIL] import fafu_robot_controller 失败: {e}")
        print("       先跑 tests/smoke_test.py 排查环境")
        return 1

    has_gripper = args.gripper_id is not None
    try:
        arm = FafuRobotController(
            cfg_path=args.cfg,
            has_gripper=has_gripper,
            gripper_motor_id=args.gripper_id,
        )
    except Exception as e:
        traceback.print_exc()
        print(f"\n[FAIL] 创建 FafuRobotController 失败: {e}")
        return 1

    app = App(arm, default_speed=args.speed)
    rc = 0
    try:
        while True:
            print(MENU)
            cmd = prompt("  >>> ").lower()
            if cmd in ("q", "quit", "exit"):
                break
            elif cmd == "s":
                app.show_state()
            elif cmd == "j":
                app.move_one_joint()
            elif cmd in ("a", "all"):
                app.move_all_joints()
            elif cmd in ("h", "home"):
                app.go_home()
            elif cmd == "g":
                app.gripper_menu()
            elif cmd == "l":
                app.limit_menu()
            elif cmd == "t":
                app.teach_record()
            elif cmd == "p":
                app.teach_replay()
            elif cmd == "m":
                app.monitor()
            elif cmd == "e":
                app.estop()
            elif cmd == "r":
                app.resume()
            elif cmd == "":
                continue
            else:
                print(f"  未识别: {cmd!r}")
    except KeyboardInterrupt:
        print("\n\n[INTERRUPT] Ctrl+C, 紧急停止...")
        try:
            arm.emergency_stop()
        except Exception:
            pass
        rc = 130
    except Exception as e:
        traceback.print_exc()
        print(f"\n[FAIL] {e}")
        try:
            arm.emergency_stop()
        except Exception:
            pass
        rc = 1
    finally:
        try:
            arm.close_connection()
        except Exception:
            pass

    return rc


if __name__ == "__main__":
    sys.exit(main())
