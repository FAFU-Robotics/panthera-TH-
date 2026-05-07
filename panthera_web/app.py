# -*- coding: utf-8 -*-
"""
panthera_web/app.py
Panthera-HT 调试板 Web 控制服务

启动:
    python app.py            # 默认 http://0.0.0.0:5000
    python app.py --port 8080
    python app.py --cfg ../Panthera-HT_SDK/panthera_cpp/motor_example_debug/robot.cfg

设计:
    - 单进程持有一个全局 HightorqueSerial 实例 (串口本身不能并发打开)
    - 所有控制 API 都用 state_lock 保护 (drv 内部本身有锁, 这里防止 connect/close 并发)
    - 连接成功后启动 50Hz 后台轮询线程缓存所有电机的状态; 前端每 100ms
      GET /api/states 拿 cache, 不会卡串口
"""
from __future__ import annotations

import argparse
import os
import sys
import threading
import time
import traceback
from typing import Any, Dict, List, Optional

from flask import Flask, jsonify, request, send_from_directory
from flask_cors import CORS

# 让 import panthera_motor 能找到本目录下的 .pyd
HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

try:
    import panthera_motor as pm  # noqa: E402
except ImportError:
    # 友好诊断: 列出当前目录下的 .pyd, 跟当前 Python 的 ABI 对比
    import glob
    pyds = sorted(glob.glob(os.path.join(HERE, "panthera_motor*.pyd")))
    abi_tag = f"cp{sys.version_info.major}{sys.version_info.minor}"
    print("=" * 70, file=sys.stderr)
    print(" 加载 panthera_motor 失败!", file=sys.stderr)
    print(f"   当前 Python:  {sys.executable}", file=sys.stderr)
    print(f"   当前版本:     {sys.version_info.major}.{sys.version_info.minor} "
          f"(ABI tag: {abi_tag})", file=sys.stderr)
    if pyds:
        print(f"   目录下找到的 .pyd: ", file=sys.stderr)
        for p in pyds:
            tag_match = abi_tag in os.path.basename(p)
            mark = "← 匹配" if tag_match else "✗ ABI 不匹配, 当前 Python 加载不了"
            print(f"     {os.path.basename(p)}  {mark}", file=sys.stderr)
        print("", file=sys.stderr)
        print(" 解决:", file=sys.stderr)
        print("   - 用对应版本的 Python 跑, 例如:", file=sys.stderr)
        print("       conda activate panthera   # Python 3.10", file=sys.stderr)
        print("       python app.py", file=sys.stderr)
        print("   - 或直接指定绝对路径:", file=sys.stderr)
        print(r"       D:\Anaconda\envs\panthera\python.exe app.py", file=sys.stderr)
        print("   - 或重新构建匹配当前 Python 版本的 .pyd: .\\build.bat", file=sys.stderr)
    else:
        print(f"   目录 {HERE} 下没有任何 panthera_motor*.pyd", file=sys.stderr)
        print(" 解决: 先在当前 Python 环境里编译模块: .\\build.bat", file=sys.stderr)
    print("=" * 70, file=sys.stderr)
    sys.exit(2)


# ============================================================================
#  单位换算辅助
# ============================================================================
_UNIT_MAP = {
    "turns":   pm.PosUnit.Turns,
    "turn":    pm.PosUnit.Turns,
    "radians": pm.PosUnit.Radians,
    "rad":     pm.PosUnit.Radians,
    "degrees": pm.PosUnit.Degrees,
    "deg":     pm.PosUnit.Degrees,
}


def parse_unit(name: Optional[str], default=pm.PosUnit.Turns):
    if not name:
        return default
    return _UNIT_MAP.get(name.lower(), default)


def unit_name(unit) -> str:
    return {
        pm.PosUnit.Turns:   "turns",
        pm.PosUnit.Radians: "radians",
        pm.PosUnit.Degrees: "degrees",
    }.get(unit, "turns")


def state_to_dict(s, unit=pm.PosUnit.Turns) -> Dict[str, Any]:
    if s is None:
        return None
    return {
        "id":       s.id,
        "mode":     s.mode,
        "fault":    s.fault,
        "position": pm.from_turns(s.position, unit),
        "position_turns": s.position,
        "velocity": s.velocity,
        "torque":   s.torque,
        "pos_limit_flag": s.pos_limit_flag,
        "unit": unit_name(unit),
    }


# ============================================================================
#  全局状态容器
# ============================================================================
class WebState:
    """整个进程只有一个; 持有 driver / cfg / 控制锁."""

    def __init__(self, default_cfg_path: Optional[str] = None):
        self.lock = threading.RLock()
        self.ht: Optional[pm.HightorqueSerial] = None
        self.cfg: Optional[pm.RobotConfig] = None
        self.cfg_path: Optional[str] = default_cfg_path
        self.port: Optional[str] = None
        self.baudrate: Optional[int] = None
        self.poll_hz: float = 50.0
        self.last_error: Optional[str] = None
        # 缓存: 用 driver 的 get_states() 拿到的最新状态字典
        self._last_can_status_str: str = ""

    # ----- connect / disconnect -----

    def connect(self,
                port: Optional[str] = None,
                baudrate: Optional[int] = None,
                cfg_path: Optional[str] = None,
                poll_hz: Optional[float] = None) -> Dict[str, Any]:
        with self.lock:
            if self.ht is not None:
                return self._info("已连接, 请先 disconnect")

            # 1) 加载配置 (失败则用内置默认)
            cfg_path = cfg_path or self.cfg_path
            cfg: Optional[pm.RobotConfig] = None
            cfg_load_msg = ""
            if cfg_path:
                try:
                    cfg = pm.RobotConfig.load(cfg_path)
                    cfg_load_msg = f"加载配置: {cfg_path}"
                except Exception as e:
                    cfg_load_msg = f"配置加载失败 ({e}), 使用默认值"

            if cfg is None:
                cfg = pm.RobotConfig()
                cfg.motor_ids = [1, 2, 3, 4, 5, 6, 7]
                cfg.pos_unit = pm.PosUnit.Turns
                cfg.limits = {
                    1: (-0.40, 0.30), 2: (-0.05, 0.48), 3: (0.00, 0.47),
                    4: (-0.25, 0.25), 5: (-0.25, 0.25), 6: (-0.25, 0.25),
                    7: (-0.25, 0.25),
                }

            # 2) 端口/波特率: 优先 HTTP 请求 > cfg
            self.port = port or cfg.port
            self.baudrate = int(baudrate or cfg.baudrate)
            self.cfg = cfg
            self.cfg_path = cfg_path
            self.poll_hz = float(poll_hz or cfg.control_rate_hz or 50.0)

            # 3) 打开串口
            try:
                self.ht = pm.HightorqueSerial(self.port, self.baudrate)
            except Exception as e:
                self.ht = None
                self.last_error = f"打开串口 {self.port} 失败: {e}"
                return self._info(self.last_error, ok=False)

            # 4) 灌限位
            try:
                cfg.apply_limits_to(self.ht)
            except Exception as e:
                self.last_error = f"应用限位失败: {e}"

            # 5) 启动后台轮询: 让 GET /api/states 永远返回最新缓存
            try:
                self.ht.start_state_polling(cfg.motor_ids, self.poll_hz)
            except Exception as e:
                self.last_error = f"启动状态轮询失败: {e}"

            self.last_error = None
            return self._info(cfg_load_msg or "已连接")

    def disconnect(self) -> Dict[str, Any]:
        with self.lock:
            if self.ht is None:
                return self._info("未连接")
            try:
                # 给所有电机发 stop, 避免下电时电机继续保持力矩
                if self.cfg is not None:
                    for mid in self.cfg.motor_ids:
                        try:
                            self.ht.stop(mid)
                        except Exception:
                            pass
                try:
                    self.ht.stop_state_polling()
                except Exception:
                    pass
                self.ht.close()
            finally:
                self.ht = None
            return self._info("已断开")

    # ----- 工具 -----

    def require_open(self):
        if self.ht is None:
            raise RuntimeError("未连接, 请先 POST /api/connect")
        return self.ht

    def _info(self, msg: str, ok: bool = True) -> Dict[str, Any]:
        return {
            "ok": ok,
            "message": msg,
            "connected": self.ht is not None,
            "port": self.port,
            "baudrate": self.baudrate,
            "cfg_path": self.cfg_path,
            "last_error": self.last_error,
        }

    def status(self) -> Dict[str, Any]:
        with self.lock:
            data = self._info("ok")
            cfg = self.cfg
            if cfg is not None:
                data["motor_ids"] = list(cfg.motor_ids)
                data["pos_unit"]  = unit_name(cfg.pos_unit)

                # limits 完全以 driver 为准 (connect 时 cfg.apply_limits_to 已灌入,
                # 之后 enable_position_limit / disable_position_limit 会动态变).
                # 没连接 driver 时, fallback 到 cfg.limits (初始值预览).
                limits_out: Dict[str, Any] = {}
                for mid in cfg.motor_ids:
                    if self.ht is not None:
                        r = None
                        try:
                            r = self.ht.get_position_limit_turns(mid)
                        except Exception:
                            pass
                        if r is None:
                            limits_out[str(mid)] = {"enabled": False}
                        else:
                            lo, hi = r
                            limits_out[str(mid)] = {
                                "enabled":  True,
                                "lo_turns": lo, "hi_turns": hi,
                                "lo":       pm.from_turns(lo, cfg.pos_unit),
                                "hi":       pm.from_turns(hi, cfg.pos_unit),
                            }
                    elif mid in cfg.limits:
                        lo, hi = cfg.limits[mid]
                        limits_out[str(mid)] = {
                            "enabled":  True,
                            "lo_turns": lo, "hi_turns": hi,
                            "lo":       pm.from_turns(lo, cfg.pos_unit),
                            "hi":       pm.from_turns(hi, cfg.pos_unit),
                        }
                    else:
                        limits_out[str(mid)] = {"enabled": False}
                data["limits"] = limits_out

                data["control_rate_hz"] = cfg.control_rate_hz
                data["max_torque_raw"]  = cfg.max_torque_raw
                data["use_async_rx"]    = cfg.use_async_rx
            else:
                data["motor_ids"] = []
                data["pos_unit"]  = "turns"
                data["limits"]    = {}

            # 加上 driver 的统计/轮询状态
            if self.ht is not None:
                try:
                    st = self.ht.get_stats()
                    data["stats"] = {
                        "tx_frames":        st.tx_frames,
                        "rx_frames":        st.rx_frames,
                        "rx_parsed":        st.rx_parsed,
                        "rx_dropped":       st.rx_dropped,
                        "last_rx_age_ms":   st.last_rx_age_ms,
                        "avg_tx_period_ms": st.avg_tx_period_ms,
                        "max_tx_jitter_ms": st.max_tx_jitter_ms,
                    }
                except Exception:
                    pass
                try:
                    data["polling"] = bool(self.ht.is_polling())
                except Exception:
                    data["polling"] = False
            return data


# ============================================================================
#  Flask App
# ============================================================================
state = WebState()
app = Flask(__name__, static_folder="static", template_folder="templates")
CORS(app)


# ----- 静态页面 -----

def _no_cache(resp):
    """禁掉浏览器缓存, 改完前端文件 F5 立即生效."""
    resp.headers["Cache-Control"] = "no-store, no-cache, must-revalidate, max-age=0"
    resp.headers["Pragma"]        = "no-cache"
    resp.headers["Expires"]       = "0"
    return resp


@app.route("/")
def index():
    return _no_cache(send_from_directory(app.template_folder, "index.html"))


@app.route("/static/<path:filename>")
def static_files(filename):
    return _no_cache(send_from_directory(app.static_folder, filename))


# ----- 串口 / 连接 -----

@app.route("/api/ports", methods=["GET"])
def api_list_ports():
    try:
        all_ports   = pm.list_serial_ports()
        candidates  = pm.find_likely_debug_boards()
        cand_set    = {p.port for p in candidates}
        return jsonify({
            "ok": True,
            "ports": [
                {
                    "port": p.port, "description": p.description,
                    "vid": p.vid, "pid": p.pid,
                    "is_candidate": p.port in cand_set,
                } for p in all_ports
            ],
        })
    except Exception as e:
        return jsonify({"ok": False, "message": str(e)}), 500


@app.route("/api/connect", methods=["POST"])
def api_connect():
    body = request.get_json(silent=True) or {}
    res = state.connect(
        port=body.get("port"),
        baudrate=body.get("baudrate"),
        cfg_path=body.get("cfg_path"),
        poll_hz=body.get("poll_hz"),
    )
    return jsonify(res), (200 if res["ok"] else 500)


@app.route("/api/disconnect", methods=["POST"])
def api_disconnect():
    res = state.disconnect()
    return jsonify(res)


@app.route("/api/status", methods=["GET"])
def api_status():
    return jsonify(state.status())


# ----- 状态读取 -----

@app.route("/api/states", methods=["GET"])
def api_states():
    """返回所有 motor_ids 的最新状态 (从 cache, 非阻塞)."""
    try:
        ht = state.require_open()
    except Exception as e:
        return jsonify({"ok": False, "message": str(e), "states": {}}), 200

    cfg = state.cfg
    motor_ids = list(cfg.motor_ids) if cfg is not None else []
    unit = cfg.pos_unit if cfg is not None else pm.PosUnit.Turns

    out: Dict[str, Any] = {}
    for mid in motor_ids:
        s = ht.get_cached_state(mid)
        out[str(mid)] = state_to_dict(s, unit) if s is not None else None
    return jsonify({
        "ok": True,
        "ts_ms": int(time.time() * 1000),
        "unit": unit_name(unit),
        "states": out,
    })


@app.route("/api/state/<int:motor_id>", methods=["GET"])
def api_state_one(motor_id: int):
    """实时读 (会发一次 CAN), 比 cache 慢 5-15ms."""
    try:
        ht = state.require_open()
    except Exception as e:
        return jsonify({"ok": False, "message": str(e)}), 400

    timeout_s = float(request.args.get("timeout_s", 0.3))
    s = ht.read_motor_state(motor_id, timeout_s)
    unit = state.cfg.pos_unit if state.cfg is not None else pm.PosUnit.Turns
    return jsonify({
        "ok": s is not None,
        "state": state_to_dict(s, unit) if s is not None else None,
        "message": "无响应" if s is None else "ok",
    })


# ----- 单个电机控制 -----

def _ctl_args() -> Dict[str, Any]:
    body = request.get_json(silent=True) or {}
    return body


@app.route("/api/motor/<int:motor_id>/move", methods=["POST"])
def api_motor_move(motor_id: int):
    """
    Body:
        {
          "pos": 0.1,                     # 必填. 目标位置, 单位由 unit 决定
          "vel_max_rps": 0.05,            # 默认 0.05
          "acc_rpss": 0.05,               # 默认 0.05
          "unit": "turns" | "radians" | "degrees"  # 默认: 跟 cfg.pos_unit
        }
    """
    try:
        ht = state.require_open()
    except Exception as e:
        return jsonify({"ok": False, "message": str(e)}), 400

    args = _ctl_args()
    if "pos" not in args:
        return jsonify({"ok": False, "message": "缺少 pos"}), 400

    default_unit = state.cfg.pos_unit if state.cfg is not None else pm.PosUnit.Turns
    unit = parse_unit(args.get("unit"), default_unit)

    pos = float(args["pos"])
    vel = float(args.get("vel_max_rps", 0.05))
    acc = float(args.get("acc_rpss",    0.05))

    try:
        s = ht.set_pos_vel_acc(motor_id, pos, vel, acc, unit)
    except Exception as e:
        return jsonify({"ok": False, "message": f"set_pos_vel_acc 异常: {e}"}), 500

    return jsonify({
        "ok": True,
        "state": state_to_dict(s, unit),
        "message": "ok" if s is not None else "已下发, 但未收到电机状态回复",
    })


@app.route("/api/motor/<int:motor_id>/set_velocity", methods=["POST"])
def api_motor_set_velocity(motor_id: int):
    try:
        ht = state.require_open()
    except Exception as e:
        return jsonify({"ok": False, "message": str(e)}), 400

    args = _ctl_args()
    vel = float(args.get("vel_rps", 0.0))
    s = ht.set_velocity(motor_id, vel)
    unit = state.cfg.pos_unit if state.cfg is not None else pm.PosUnit.Turns
    return jsonify({"ok": True, "state": state_to_dict(s, unit)})


@app.route("/api/motor/<int:motor_id>/stop", methods=["POST"])
def api_motor_stop(motor_id: int):
    try:
        ht = state.require_open()
    except Exception as e:
        return jsonify({"ok": False, "message": str(e)}), 400
    s = ht.stop(motor_id)
    unit = state.cfg.pos_unit if state.cfg is not None else pm.PosUnit.Turns
    return jsonify({"ok": True, "state": state_to_dict(s, unit), "message": "已停止 (PWM off)"})


@app.route("/api/motor/<int:motor_id>/brake", methods=["POST"])
def api_motor_brake(motor_id: int):
    try:
        ht = state.require_open()
    except Exception as e:
        return jsonify({"ok": False, "message": str(e)}), 400
    s = ht.brake(motor_id)
    unit = state.cfg.pos_unit if state.cfg is not None else pm.PosUnit.Turns
    return jsonify({"ok": True, "state": state_to_dict(s, unit), "message": "已刹车"})


@app.route("/api/motor/<int:motor_id>/reset_zero", methods=["POST"])
def api_motor_reset_zero(motor_id: int):
    """危险: 把当前位置写为零点. 需 body 里带 confirm=true."""
    try:
        ht = state.require_open()
    except Exception as e:
        return jsonify({"ok": False, "message": str(e)}), 400
    args = _ctl_args()
    if not args.get("confirm"):
        return jsonify({"ok": False, "message": "请在 body 里带 confirm=true 确认"}), 400
    raw = ht.reset_zero(motor_id)
    return jsonify({"ok": True, "raw": raw, "message": "已发送 set_zero"})


@app.route("/api/motor/<int:motor_id>/save_config", methods=["POST"])
def api_motor_save_config(motor_id: int):
    try:
        ht = state.require_open()
    except Exception as e:
        return jsonify({"ok": False, "message": str(e)}), 400
    raw = ht.save_config(motor_id)
    return jsonify({"ok": True, "raw": raw, "message": "已发送 save_config"})


@app.route("/api/motor/<int:motor_id>/version", methods=["GET", "POST"])
def api_motor_version(motor_id: int):
    try:
        ht = state.require_open()
    except Exception as e:
        return jsonify({"ok": False, "message": str(e)}), 400
    v = ht.read_version(motor_id)
    return jsonify({"ok": v is not None, "version": v or "", "message": "ok" if v else "无回复"})


@app.route("/api/motor/<int:motor_id>/motor_reset", methods=["POST"])
def api_motor_reset(motor_id: int):
    try:
        ht = state.require_open()
    except Exception as e:
        return jsonify({"ok": False, "message": str(e)}), 400
    args = _ctl_args()
    if not args.get("confirm"):
        return jsonify({"ok": False, "message": "请在 body 里带 confirm=true 确认"}), 400
    raw = ht.motor_reset(motor_id)
    return jsonify({"ok": True, "raw": raw, "message": "已发送 motor_reset"})


# ----- 全局批量控制 -----

@app.route("/api/stop_all", methods=["POST"])
def api_stop_all():
    try:
        ht = state.require_open()
    except Exception as e:
        return jsonify({"ok": False, "message": str(e)}), 400
    cfg = state.cfg
    if cfg is None:
        return jsonify({"ok": False, "message": "未加载配置"}), 400
    ok = 0
    fail: List[int] = []
    for mid in cfg.motor_ids:
        try:
            ht.stop(mid)
            ok += 1
        except Exception:
            fail.append(mid)
    return jsonify({"ok": True, "stopped": ok, "failed": fail, "message": f"全部停止 (ok={ok})"})


@app.route("/api/home_all", methods=["POST"])
def api_home_all():
    """所有电机回到 0 (单位用 cfg.pos_unit). 用 set_many_pos_vel_tqe 一帧搞定."""
    try:
        ht = state.require_open()
    except Exception as e:
        return jsonify({"ok": False, "message": str(e)}), 400
    cfg = state.cfg
    if cfg is None:
        return jsonify({"ok": False, "message": "未加载配置"}), 400

    args = _ctl_args()
    vel_max = float(args.get("vel_max_rps", 0.05))
    cmds = [pm.ManyMotorCmd(mid, 0.0, vel_max, 0) for mid in cfg.motor_ids]
    try:
        states = ht.set_many_pos_vel_tqe(cmds, pm.PosUnit.Turns,
                                         max(cfg.motor_ids), 0.05)
    except Exception as e:
        return jsonify({"ok": False, "message": f"set_many 失败: {e}"}), 500
    unit = cfg.pos_unit
    return jsonify({
        "ok": True,
        "message": f"已下发 home (vel_max={vel_max})",
        "states": {str(mid): state_to_dict(s, unit) for mid, s in states.items()},
    })


@app.route("/api/move_many", methods=["POST"])
def api_move_many():
    """
    Body:
        {
          "vel_max_rps": 0.05,
          "unit": "turns" | "radians" | "degrees",
          "targets": { "1": 0.1, "2": 0.0, ... }
        }
    """
    try:
        ht = state.require_open()
    except Exception as e:
        return jsonify({"ok": False, "message": str(e)}), 400
    cfg = state.cfg
    if cfg is None:
        return jsonify({"ok": False, "message": "未加载配置"}), 400

    args = _ctl_args()
    vel_max = float(args.get("vel_max_rps", 0.05))
    unit = parse_unit(args.get("unit"), cfg.pos_unit)
    targets = args.get("targets") or {}
    cmds = []
    for k, v in targets.items():
        try:
            mid = int(k)
            pos = float(v)
        except Exception:
            continue
        cmds.append(pm.ManyMotorCmd(mid, pos, vel_max, 0))
    if not cmds:
        return jsonify({"ok": False, "message": "targets 为空"}), 400
    try:
        states = ht.set_many_pos_vel_tqe(cmds, unit,
                                         max(cfg.motor_ids), 0.05)
    except Exception as e:
        return jsonify({"ok": False, "message": str(e)}), 500
    return jsonify({
        "ok": True,
        "message": f"已下发 {len(cmds)} 个目标",
        "states": {str(mid): state_to_dict(s, cfg.pos_unit) for mid, s in states.items()},
    })


# ============================================================================
#  限位标定 (运行时修改 + 落盘)
# ============================================================================

def _current_limit_turns(mid: int) -> Optional[tuple]:
    """读 driver 当前限位 (圈); 返回 (lo, hi) 或 None."""
    if state.ht is None:
        return None
    try:
        return state.ht.get_position_limit_turns(mid)
    except Exception:
        return None


def _format_limit_response(mid: int, msg: str = "ok") -> Dict[str, Any]:
    cfg_unit = state.cfg.pos_unit if state.cfg is not None else pm.PosUnit.Turns
    r = _current_limit_turns(mid)
    if r is None:
        return {"ok": True, "message": msg, "motor_id": mid, "enabled": False}
    lo, hi = r
    return {
        "ok": True, "message": msg, "motor_id": mid, "enabled": True,
        "lo_turns": lo, "hi_turns": hi,
        "lo": pm.from_turns(lo, cfg_unit),
        "hi": pm.from_turns(hi, cfg_unit),
        "unit": unit_name(cfg_unit),
    }


@app.route("/api/motor/<int:motor_id>/limit", methods=["POST"])
def api_motor_limit_set(motor_id: int):
    """
    部分更新软限位.

    Body 任选其一/全部 (单位由 unit 决定, 默认 cfg.pos_unit):
        { "lo": -0.4 }                 # 只改下限
        { "hi": 0.3 }                  # 只改上限
        { "lo": -0.4, "hi": 0.3 }      # 一起改
        { "lo": -22.9, "unit": "deg" } # 用度数

    没在 body 里的字段保留原值; 如果原本没设过限位且只给一个字段, 报错.
    """
    try:
        ht = state.require_open()
    except Exception as e:
        return jsonify({"ok": False, "message": str(e)}), 400

    args = _ctl_args()
    cfg_unit = state.cfg.pos_unit if state.cfg is not None else pm.PosUnit.Turns
    unit = parse_unit(args.get("unit"), cfg_unit)

    cur = _current_limit_turns(motor_id)
    cur_lo, cur_hi = (cur if cur is not None else (None, None))

    # 把 body 里给的换算成圈
    new_lo = pm.to_turns(float(args["lo"]), unit) if "lo" in args and args["lo"] is not None else cur_lo
    new_hi = pm.to_turns(float(args["hi"]), unit) if "hi" in args and args["hi"] is not None else cur_hi

    if new_lo is None or new_hi is None:
        return jsonify({
            "ok": False,
            "message": "电机原本未设限位, 必须同时给 lo 和 hi"
        }), 400
    if new_lo > new_hi:
        return jsonify({"ok": False, "message": f"lo({new_lo}) > hi({new_hi})"}), 400

    try:
        ht.enable_position_limit(motor_id, new_lo, new_hi, pm.PosUnit.Turns)
    except Exception as e:
        return jsonify({"ok": False, "message": f"enable_position_limit 失败: {e}"}), 500

    # 同步写到 cfg.limits, 这样下次 GET /api/status 也是新的
    if state.cfg is not None:
        state.cfg.limits[motor_id] = (new_lo, new_hi)

    return jsonify(_format_limit_response(motor_id, "已更新限位"))


@app.route("/api/motor/<int:motor_id>/limit/set_current_as", methods=["POST"])
def api_motor_limit_set_current_as(motor_id: int):
    """
    把当前电机位置作为新的 lo 或 hi.

    Body:
        { "which": "lo" }     # 当前位置 → 下限
        { "which": "hi" }     # 当前位置 → 上限

    优先用 cache 里的最新位置 (后台轮询线程产物); cache 没有则实时读一次.
    设新限位时:
        - 旧的另一端保留;
        - 没设过限位时, 把另一端先设到 ±10 圈作为占位 (随后用户应 set_current_as 另一端).
    """
    try:
        ht = state.require_open()
    except Exception as e:
        return jsonify({"ok": False, "message": str(e)}), 400

    args = _ctl_args()
    which = (args.get("which") or "").lower()
    if which not in ("lo", "hi"):
        return jsonify({"ok": False, "message": "which 必须是 'lo' 或 'hi'"}), 400

    # 1) 读当前位置 (优先 cache)
    s = ht.get_cached_state(motor_id)
    if s is None:
        s = ht.read_motor_state(motor_id, 0.3)
    if s is None:
        return jsonify({"ok": False, "message": "无法读取电机当前位置 (无响应)"}), 502

    pos_turns = float(s.position)

    # 2) 跟原 hi/lo 拼成新的 (lo, hi)
    cur = _current_limit_turns(motor_id)
    if cur is None:
        # 没设过限位, 给一个保守的"伸缩域", 后续用户再 set_current_as 另一端
        FAR = 10.0
        cur_lo, cur_hi = (-FAR, FAR)
    else:
        cur_lo, cur_hi = cur

    if which == "lo":
        new_lo, new_hi = pos_turns, cur_hi
    else:
        new_lo, new_hi = cur_lo, pos_turns

    if new_lo > new_hi:
        return jsonify({
            "ok": False,
            "message": f"新 {which}={pos_turns:.4f} 圈 与另一端冲突: "
                       f"lo={new_lo:.4f} > hi={new_hi:.4f}. 请先调另一端."
        }), 400

    try:
        ht.enable_position_limit(motor_id, new_lo, new_hi, pm.PosUnit.Turns)
    except Exception as e:
        return jsonify({"ok": False, "message": f"enable_position_limit 失败: {e}"}), 500

    if state.cfg is not None:
        state.cfg.limits[motor_id] = (new_lo, new_hi)

    return jsonify(_format_limit_response(
        motor_id,
        f"已把电机 {motor_id} 当前位置 {pos_turns:.4f} 圈 设为 {which}"
    ))


@app.route("/api/motor/<int:motor_id>/limit/disable", methods=["POST"])
def api_motor_limit_disable(motor_id: int):
    """禁用单关节软限位 (driver 不再 clamp; cfg.limits 也清掉)."""
    try:
        ht = state.require_open()
    except Exception as e:
        return jsonify({"ok": False, "message": str(e)}), 400
    try:
        ht.disable_position_limit(motor_id)
    except Exception as e:
        return jsonify({"ok": False, "message": str(e)}), 500
    if state.cfg is not None and motor_id in state.cfg.limits:
        del state.cfg.limits[motor_id]
    return jsonify({"ok": True, "message": f"已禁用电机 {motor_id} 的软限位",
                    "motor_id": motor_id, "enabled": False})


@app.route("/api/limits/clear", methods=["POST"])
def api_limits_clear_all():
    """清空全部软限位."""
    try:
        ht = state.require_open()
    except Exception as e:
        return jsonify({"ok": False, "message": str(e)}), 400
    ht.clear_all_position_limits()
    if state.cfg is not None:
        state.cfg.limits.clear()
    return jsonify({"ok": True, "message": "已清空全部软限位"})


def _save_limits_to_cfg_file(path: str) -> Dict[str, Any]:
    """
    把 driver 当前 limits 写回 cfg 文件:
      - 保留原文件其它行 (port / baudrate / motor_ids / 注释 / 空行 ...)
      - 替换所有 limits.* 行
      - 用 cfg.pos_unit 做单位 (跟原 cfg 风格一致)
      - 如果原文件没有"# limits"小节, 末尾新增
    """
    cfg = state.cfg
    if cfg is None:
        raise RuntimeError("未加载配置, 没法保存")
    if state.ht is None:
        raise RuntimeError("未连接, 无法读 driver 限位")

    # 1) 收集 driver 端当前所有限位 (圈), 按 motor_id 排序
    lims_turns: Dict[int, tuple] = {}
    for mid in cfg.motor_ids:
        r = _current_limit_turns(mid)
        if r is not None:
            lims_turns[mid] = r

    # 2) 按原 cfg.pos_unit 换算并格式化成新行
    out_unit = cfg.pos_unit
    unit_str = unit_name(out_unit)

    def fmt_pair(mid: int, lo_t: float, hi_t: float) -> str:
        lo = pm.from_turns(lo_t, out_unit)
        hi = pm.from_turns(hi_t, out_unit)
        return f"limits.{mid:<3} = {lo:>9.4f}, {hi:>9.4f}"

    new_limit_lines = [fmt_pair(mid, *lims_turns[mid]) for mid in sorted(lims_turns)]

    # 3) 读原文件 (允许不存在: 直接整文件重写一个最小版)
    if not os.path.exists(path):
        body = (
            "# robot.cfg (由 panthera_web 自动生成)\n"
            f"port      = {cfg.port}\n"
            f"baudrate  = {cfg.baudrate}\n"
            f"motor_ids = {', '.join(str(m) for m in cfg.motor_ids)}\n"
            f"pos_unit  = {unit_str}\n\n"
            + "\n".join(new_limit_lines) + "\n"
        )
        with open(path, "w", encoding="utf-8") as f:
            f.write(body)
        return {"ok": True, "message": f"新建了配置文件: {path}", "path": path,
                "saved_limits": len(new_limit_lines)}

    # 4) 已存在: 行级处理
    with open(path, "r", encoding="utf-8") as f:
        old_lines = f.read().splitlines()

    new_lines: List[str] = []
    seen_limits = False
    for line in old_lines:
        # 判断这一行是不是 "limits.<id> = ..." (允许前面空白和行尾注释)
        s = line.strip()
        is_limit_line = False
        if s and not s.startswith("#") and not s.startswith(";"):
            head = s.split("=", 1)[0].strip().lower()
            if head.startswith("limits."):
                is_limit_line = True
        if is_limit_line:
            if not seen_limits:
                # 在第一条 limits.* 出现的位置, 一次性写入所有新 limits, 然后删原行
                new_lines.extend(new_limit_lines)
                seen_limits = True
            # else: 后续 limits.* 行直接吃掉, 不写
        else:
            new_lines.append(line)

    if not seen_limits and new_limit_lines:
        if new_lines and new_lines[-1].strip() != "":
            new_lines.append("")
        new_lines.append("# ---- 软限位 (panthera_web 自动写入) ----")
        new_lines.extend(new_limit_lines)

    body = "\n".join(new_lines)
    if not body.endswith("\n"):
        body += "\n"

    # 5) 写回; 顺便备份原文件一份
    bak = path + ".bak"
    try:
        if os.path.exists(path):
            with open(path, "r", encoding="utf-8") as f_in, \
                 open(bak,  "w", encoding="utf-8") as f_out:
                f_out.write(f_in.read())
    except Exception:
        bak = None

    with open(path, "w", encoding="utf-8") as f:
        f.write(body)

    return {
        "ok": True,
        "message": f"已保存 {len(new_limit_lines)} 条限位到 {path}",
        "path": path,
        "backup": bak,
        "saved_limits": len(new_limit_lines),
        "unit": unit_str,
    }


@app.route("/api/limits/save", methods=["POST"])
def api_limits_save():
    """
    把 driver 当前所有限位写回 robot.cfg 文件 (重启后还在).

    Body (可选):
        { "path": "path/to/your.cfg" }   # 默认 state.cfg_path

    会先把原文件备份成 <path>.bak.
    """
    args = _ctl_args()
    path = args.get("path") or state.cfg_path
    if not path:
        return jsonify({"ok": False, "message": "未提供 path 且 state.cfg_path 为空"}), 400
    try:
        return jsonify(_save_limits_to_cfg_file(path))
    except Exception as e:
        traceback.print_exc()
        return jsonify({"ok": False, "message": str(e)}), 500


# ----- CAN 状态 -----

@app.route("/api/can_status", methods=["GET"])
def api_can_status():
    try:
        ht = state.require_open()
    except Exception as e:
        return jsonify({"ok": False, "message": str(e)}), 400
    st = ht.read_can_status()
    fault_name = {
        pm.CanFault.Unknown:      "Unknown",
        pm.CanFault.Ok:           "Ok",
        pm.CanFault.ErrorWarning: "ErrorWarning",
        pm.CanFault.ErrorPassive: "ErrorPassive",
        pm.CanFault.BusOff:       "BusOff",
    }.get(st.fault, "?")
    return jsonify({
        "ok": st.is_ok(),
        "fault":        fault_name,
        "lec":          st.lec,
        "tx_err_count": st.tx_err_count,
        "rx_err_count": st.rx_err_count,
        "raw":          st.raw,
    })


# ----- 通用错误处理 -----

@app.errorhandler(Exception)
def _on_error(e):
    traceback.print_exc()
    return jsonify({"ok": False, "message": f"{type(e).__name__}: {e}"}), 500


# ============================================================================
#  入口
# ============================================================================
def main():
    parser = argparse.ArgumentParser(description="Panthera-HT Web Control")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--cfg",  default=None,
                        help="默认配置文件路径 (POST /api/connect 不传 cfg_path 时用此)")
    parser.add_argument("--debug", action="store_true")
    args = parser.parse_args()

    # 默认尝试用 motor_example_debug 自带的 robot.cfg
    if args.cfg is None:
        guess = os.path.normpath(os.path.join(
            HERE, "..", "Panthera-HT_SDK", "panthera_cpp",
            "motor_example_debug", "robot.cfg"))
        if os.path.exists(guess):
            args.cfg = guess

    state.cfg_path = args.cfg
    print("=" * 60)
    print(" Panthera-HT Web Control")
    print(f"   监听:        http://{args.host}:{args.port}/")
    print(f"   默认配置:    {args.cfg or '(无, 将使用内置默认)'}")
    print(f"   pyd:         {pm.__file__}")
    print("=" * 60)

    # 关 reloader, 不然会 spawn 第二个进程, 串口被两次打开会冲突
    app.run(host=args.host, port=args.port, debug=args.debug,
            use_reloader=False, threaded=True)


if __name__ == "__main__":
    main()
