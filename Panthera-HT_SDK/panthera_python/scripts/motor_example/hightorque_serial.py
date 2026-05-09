"""
Panthera-HT 纯 Python 串口驱动 — 通过调试板的 ASCII 命令协议控制电机。

调试板 USB 串口协议:
  发送 CAN 帧: can send <CAN_ID_hex> <DATA_hex>\r\n  →  回复 OK\r\n
  查询状态:    can status\r\n  →  回复 lec=... err=... ...\r\n
  查询配置:    can config\r\n  →  回复 clk=... np=... ...\r\n

CAN 帧协议 (livelybot FDCAN):
  CAN ID: 0x8000 | motor_id  (高位=1 表示需要回复)
  Payload: 由子帧组成, 每个子帧 = cmd + addr + data...
    cmd 编码: [7:4]=读写类型, [3:2]=数据类型, [1:0]=数据个数/模式

单位 (int16):
  位置: 0.0001 转 (5000 = 0.5圈)
  速度: 0.00025 转/秒 (400 = 0.1转/秒)
  力矩: 原始值, 需乘以力矩系数
"""
from __future__ import annotations

import struct
import time
import math
import threading
import serial


# ---------------------------------------------------------------------------
#  常量
# ---------------------------------------------------------------------------

NAN_INT16 = -32768  # 0x8000, int16 "无限制" 标记
NAN_INT32 = 0x80000000
PADDING = 0x50

# CAN-FD DLC 对应的有效字节数
CANFD_DLC_SIZES = [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64]


def canfd_pad(data: bytes) -> bytes:
    """按 CAN-FD DLC 规则补 0x50 填充。"""
    n = len(data)
    for size in CANFD_DLC_SIZES:
        if size >= n:
            return data + bytes([PADDING] * (size - n))
    return data + bytes([PADDING] * (64 - n))


# ---------------------------------------------------------------------------
#  int16 单位转换 (协议文档 2.6/2.7)
# ---------------------------------------------------------------------------

def turns_to_int16(turns: float) -> int:
    """位置: 圈 → int16 (LSB = 0.0001 转)"""
    return max(-32768, min(32767, int(turns / 0.0001)))

def int16_to_turns(val: int) -> float:
    """位置: int16 → 圈"""
    return val * 0.0001

def rps_to_int16(rps: float) -> int:
    """速度: 转/秒 → int16 (LSB = 0.00025 转/秒)"""
    return max(-32768, min(32767, int(rps / 0.00025)))

def int16_to_rps(val: int) -> float:
    """速度: int16 → 转/秒"""
    return val * 0.00025

def rad_to_int16(rad: float) -> int:
    """位置: 弧度 → int16"""
    return turns_to_int16(rad / (2 * math.pi))

def int16_to_rad(val: int) -> float:
    """位置: int16 → 弧度"""
    return int16_to_turns(val) * 2 * math.pi

def rad_s_to_int16(rad_s: float) -> int:
    """速度: rad/s → int16"""
    return rps_to_int16(rad_s / (2 * math.pi))

def int16_to_rad_s(val: int) -> float:
    """速度: int16 → rad/s"""
    return int16_to_rps(val) * 2 * math.pi

# 力矩系数表 (文档 2.3)
TORQUE_COEFF = {
    "M3536_32": 0.458105,
    "M4438_30": 0.5256,
    "M4438_32": 0.485565,
    "M4538_19": 0.493835,
    "M5043_20": 0.966,
    "M5046_20": 0.533654,
    "M5047_09": 0.547474,
    "M5047_36": 0.803,
    "M6056_36": 0.677,
    "M7256_35": 0.676524,
    "M60SG_35": 0.7942,
    "M60BM_35": 0.7942,
}


# ---------------------------------------------------------------------------
#  CAN 帧 payload 构建 (基于 livelybot_fdcan.c)
# ---------------------------------------------------------------------------

def _query_subframe_int16() -> bytes:
    """查询电机状态的子帧 (附加在控制帧末尾)。
    0x14: read, int16, mode2; 0x04: 4个数据; 0x00: 起始地址0
    0x11: read, int8, 1个数据; 0x0f: 寄存器地址0x0f (故障码)
    """
    return bytes([0x14, 0x04, 0x00, 0x11, 0x0f])


def build_read_state_int16() -> bytes:
    """读取电机状态 (模式, 位置, 速度, 力矩, 故障码)"""
    return _query_subframe_int16()


def build_stop_int16() -> bytes:
    """停止电机 (三相悬空, 自由转动)"""
    return canfd_pad(bytes([0x01, 0x00, 0x00]) + _query_subframe_int16())


def build_brake_int16() -> bytes:
    """刹车 (三相短接到地)"""
    return canfd_pad(bytes([0x01, 0x00, 0x0f]) + _query_subframe_int16())


def build_pos_int16(pos: int) -> bytes:
    """位置控制 (int16, 单位 0.0001 转)"""
    payload = bytes([0x01, 0x00, 0x0A, 0x05, 0x20])
    payload += struct.pack("<h", pos)
    payload += _query_subframe_int16()
    return canfd_pad(payload)


def build_vel_int16(vel: int) -> bytes:
    """速度控制 (int16, 单位 0.00025 转/秒)"""
    payload = bytes([0x01, 0x00, 0x0A, 0x06, 0x20])
    payload += struct.pack("<h", NAN_INT16)  # 位置=无限制
    payload += struct.pack("<h", vel)
    payload += _query_subframe_int16()
    return canfd_pad(payload)


def build_pos_vel_tqe_int16(pos: int, vel: int, tqe: int) -> bytes:
    """位置+速度+最大力矩控制 (int16)"""
    payload = bytes([0x01, 0x00, 0x0a, 0x06, 0x20])
    payload += struct.pack("<h", NAN_INT16)  # 位置先占位
    payload += struct.pack("<h", vel)
    payload += bytes([0x06, 0x25])
    payload += struct.pack("<h", tqe)
    payload += struct.pack("<h", pos)
    payload += _query_subframe_int16()
    return canfd_pad(payload)


def build_pos_velmax_acc_int16(pos: int, vel_max: int, acc: int) -> bytes:
    """梯形控制: 位置+最大速度+加速度 (int16)"""
    payload = bytes([0x01, 0x00, 0x0A, 0x05, 0x20])
    payload += struct.pack("<h", pos)
    payload += bytes([0x06, 0x28])
    payload += struct.pack("<h", vel_max)
    payload += struct.pack("<h", acc)
    payload += bytes([0x00, 0x00])  # reserved
    payload += _query_subframe_int16()
    return canfd_pad(payload)


def build_torque_int16(tqe: int) -> bytes:
    """力矩控制 (int16)。参考 set_torque_int16。"""
    payload = bytes([0x01, 0x00, 0x0a, 0x04, 0x06, 0x20, 0x00, 0x80])
    # 速度=NAN, 力矩, kp=NAN, kd=0, 最大力矩=NAN
    payload += bytes([0x00, 0x00])  # vel = 0
    payload += struct.pack("<h", tqe)
    payload += bytes([0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80])
    payload += _query_subframe_int16()
    return canfd_pad(payload)


def build_voltage_int16(volt: int) -> bytes:
    """DQ 电压控制 (int16, 单位 0.1V)。参考 set_dq_volt_int16。"""
    payload = bytes([0x01, 0x00, 0x08, 0x06, 0x1a, 0x00, 0x00])
    payload += struct.pack("<h", volt)
    payload += _query_subframe_int16()
    return canfd_pad(payload)


def build_current_int16(cur: int) -> bytes:
    """DQ 电流控制 (int16, 单位 0.1A)。参考 set_dq_current_int16。"""
    payload = bytes([0x01, 0x00, 0x09, 0x06, 0x1C])
    payload += struct.pack("<h", cur)
    payload += bytes([0x00, 0x00])  # d电流 = 0
    payload += _query_subframe_int16()
    return canfd_pad(payload)


def build_pos_vel_tqe_kp_kd_int16(pos: int, vel: int, tqe: int, kp: int, kd: int) -> bytes:
    """真运控模式 (int16): 输出力矩 = (pos偏差)*kp + (vel偏差)*kd + tqe。
    参考 set_pos_vel_tqe_kp_kd_int16_2。"""
    payload = bytes([0x01, 0x00, 0x15, 0x07, 0x20])
    payload += struct.pack("<h", pos)
    payload += struct.pack("<h", vel)
    payload += struct.pack("<h", tqe)
    payload += bytes([0x06, 0x2b])
    payload += struct.pack("<h", kp)
    payload += struct.pack("<h", kd)
    payload += _query_subframe_int16()
    return canfd_pad(payload)


def build_motor_reset() -> bytes:
    """电机软重启"""
    return bytes([0x40, 0x01, 0x08, 0x64, 0x20, 0x72, 0x65, 0x73, 0x65, 0x74, 0x0A, 0x50])


def build_conf_write() -> bytes:
    """保存电机设置"""
    return canfd_pad(bytes([0x40, 0x01, 0x0B, 0x63, 0x6F, 0x6E, 0x66, 0x20, 0x77, 0x72, 0x69, 0x74, 0x65, 0x0A]))


def build_set_zero() -> bytes:
    """重设电机零位"""
    return bytes([0x40, 0x01, 0x15, 0x64, 0x20, 0x63, 0x66, 0x67, 0x2d, 0x73,
                  0x65, 0x74, 0x2d, 0x6f, 0x75, 0x74, 0x70, 0x75, 0x74, 0x20,
                  0x30, 0x2e, 0x30, 0x0a])


def build_read_version() -> bytes:
    """查询电机固件版本"""
    return bytes([0x15, 0xB5, 0x02])


def build_set_timeout_int16(timeout_ms: int) -> bytes:
    """设置电机超时时间 (ms)"""
    payload = bytes([0x05, 0x1f])
    payload += struct.pack("<h", timeout_ms)
    return payload


# ---------------------------------------------------------------------------
#  CAN 回复解析
# ---------------------------------------------------------------------------

class MotorState:
    __slots__ = ("id", "mode", "fault", "position", "velocity", "torque")
    def __init__(self, id=0, mode=0, fault=0, position=0.0, velocity=0.0, torque=0.0):
        self.id = id
        self.mode = mode
        self.fault = fault
        self.position = position
        self.velocity = velocity
        self.torque = torque

    def __repr__(self):
        return (f"MotorState(id={self.id}, mode={self.mode}, fault=0x{self.fault:02X}, "
                f"pos={self.position:.4f} turns, vel={self.velocity:.4f} rps, "
                f"tqe_raw={self.torque:.1f})")


def parse_motor_state_int16(can_data: bytes) -> MotorState | None:
    """
    解析电机回复的 int16 状态帧。
    回复格式 (模式二): 0x24 0x04 0x00 <mode_lo mode_hi> <pos_lo pos_hi> <vel_lo vel_hi> <tqe_lo tqe_hi> [0x21 0x0f <fault>] [0x50...]
    """
    if len(can_data) < 2:
        return None

    state = MotorState()
    offset = 0

    while offset < len(can_data):
        if can_data[offset] == PADDING:
            offset += 1
            continue

        cmd = can_data[offset]
        op = (cmd >> 4) & 0x0F
        dtype = (cmd >> 2) & 0x03
        count = cmd & 0x03

        if op != 2:  # 不是回复帧
            break

        offset += 1
        if offset >= len(can_data):
            break

        if count == 0:  # 模式二
            if offset >= len(can_data):
                break
            num = can_data[offset]
            offset += 1
            if offset >= len(can_data):
                break
            addr = can_data[offset]
            offset += 1

            if dtype == 1:  # int16
                for i in range(num):
                    if offset + 2 > len(can_data):
                        break
                    val = struct.unpack_from("<h", can_data, offset)[0]
                    offset += 2
                    reg = addr + i
                    if reg == 0x00:
                        state.mode = val
                    elif reg == 0x01:
                        state.position = int16_to_turns(val)
                    elif reg == 0x02:
                        state.velocity = int16_to_rps(val)
                    elif reg == 0x03:
                        state.torque = val
            elif dtype == 2:  # int32
                for i in range(num):
                    if offset + 4 > len(can_data):
                        break
                    val = struct.unpack_from("<i", can_data, offset)[0]
                    offset += 4
                    reg = addr + i
                    if reg == 0x00:
                        state.mode = val
                    elif reg == 0x01:
                        state.position = val * 0.00001
                    elif reg == 0x02:
                        state.velocity = val * 0.00001
                    elif reg == 0x03:
                        state.torque = val
            elif dtype == 0:  # int8
                for i in range(num):
                    if offset + 1 > len(can_data):
                        break
                    val = struct.unpack_from("<b", can_data, offset)[0]
                    offset += 1
                    reg = addr + i
                    if reg == 0x0f:
                        state.fault = val & 0xFF
        else:  # 模式一
            addr = can_data[offset]
            offset += 1
            type_size = [1, 2, 4, 4][dtype]
            fmt = ["<b", "<h", "<i", "<f"][dtype]
            for i in range(count):
                if offset + type_size > len(can_data):
                    break
                val = struct.unpack_from(fmt, can_data, offset)[0]
                offset += type_size
                reg = addr + i
                if reg == 0x00:
                    state.mode = int(val)
                elif reg == 0x01:
                    if dtype == 1:
                        state.position = int16_to_turns(int(val))
                    elif dtype == 2:
                        state.position = val * 0.00001
                    else:
                        state.position = val
                elif reg == 0x02:
                    if dtype == 1:
                        state.velocity = int16_to_rps(int(val))
                    elif dtype == 2:
                        state.velocity = val * 0.00001
                    else:
                        state.velocity = val
                elif reg == 0x03:
                    state.torque = val
                elif reg == 0x0f:
                    state.fault = int(val) & 0xFF

    return state


# ---------------------------------------------------------------------------
#  调试板串口驱动
# ---------------------------------------------------------------------------

class HightorqueSerial:
    """通过调试板的 ASCII 命令协议控制电机。"""

    def __init__(self, port: str, baudrate: int = 4000000):
        self.ser = serial.Serial(port, baudrate, timeout=0.1)
        self._lock = threading.Lock()
        time.sleep(0.1)
        self.ser.reset_input_buffer()

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    # -- 底层串口通信 --

    def _serial_write(self, data: bytes, retries: int = 3):
        """带重试的串口写入 (处理 Windows USB 串口瞬时错误)。"""
        for attempt in range(retries):
            try:
                self.ser.write(data)
                self.ser.flush()
                return
            except serial.SerialException:
                if attempt == retries - 1:
                    raise
                time.sleep(0.05)
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()

    def _send_cmd(self, cmd: str, timeout: float = 0.5) -> str:
        """发送文本命令，读取回复直到超时。"""
        with self._lock:
            self.ser.reset_input_buffer()
            self._serial_write((cmd + "\r\n").encode())

            response = b""
            deadline = time.time() + timeout
            while time.time() < deadline:
                n = self.ser.in_waiting
                if n > 0:
                    response += self.ser.read(n)
                    if b"\n" in response:
                        break
                else:
                    time.sleep(0.005)

            return response.decode("ascii", errors="replace").strip()

    def _send_can(self, can_id: int, data: bytes, timeout: float = 0.5) -> str:
        """发送 CAN 帧: can send <ID_hex> <DATA_hex>"""
        id_hex = f"{can_id:X}"
        data_hex = data.hex().upper()
        cmd = f"can send {id_hex} {data_hex}"
        return self._send_cmd(cmd, timeout)

    def _send_can_and_recv(self, can_id: int, data: bytes, timeout: float = 0.5) -> list[tuple[int, bytes]]:
        """
        发送 CAN 帧并等待回复。
        返回收到的 CAN 帧列表: [(reply_can_id, reply_data_bytes), ...]
        """
        with self._lock:
            self.ser.reset_input_buffer()
            id_hex = f"{can_id:X}"
            data_hex = data.hex().upper()
            cmd = f"can send {id_hex} {data_hex}\r\n"
            self._serial_write(cmd.encode())

            response = b""
            deadline = time.time() + timeout
            while time.time() < deadline:
                n = self.ser.in_waiting
                if n > 0:
                    response += self.ser.read(n)
                    if response.count(b"\n") >= 2:
                        break
                time.sleep(0.01)

            return self._parse_rcv_lines(response.decode("ascii", errors="replace"))

    @staticmethod
    def _parse_rcv_lines(text: str) -> list[tuple[int, bytes]]:
        """解析 rcv 回复行。格式: rcv <ID_hex> <DATA_hex> <flags...>"""
        results = []
        for line in text.split("\n"):
            line = line.strip()
            if not line.startswith("rcv "):
                continue
            parts = line.split()
            if len(parts) < 3:
                continue
            try:
                rcv_id = int(parts[1], 16)
                rcv_data = bytes.fromhex(parts[2])
                results.append((rcv_id, rcv_data))
            except (ValueError, IndexError):
                continue
        return results

    # -- 调试板命令 --

    def can_status(self) -> str:
        return self._send_cmd("can status")

    def can_config(self) -> str:
        return self._send_cmd("can config")

    # -- 电机控制 --

    def read_motor_state(self, motor_id: int, timeout: float = 0.5) -> MotorState | None:
        """读取单个电机的状态。"""
        can_id = 0x8000 | motor_id
        payload = build_read_state_int16()
        replies = self._send_can_and_recv(can_id, payload, timeout)
        for rcv_id, rcv_data in replies:
            state = parse_motor_state_int16(rcv_data)
            if state is not None:
                state.id = motor_id
                return state
        return None

    def stop(self, motor_id: int) -> MotorState | None:
        """停止电机，返回电机状态。"""
        can_id = 0x8000 | motor_id
        replies = self._send_can_and_recv(can_id, build_stop_int16())
        for rcv_id, rcv_data in replies:
            state = parse_motor_state_int16(rcv_data)
            if state is not None:
                state.id = motor_id
                return state
        return None

    def brake(self, motor_id: int) -> MotorState | None:
        """刹车，返回电机状态。"""
        can_id = 0x8000 | motor_id
        replies = self._send_can_and_recv(can_id, build_brake_int16())
        for rcv_id, rcv_data in replies:
            state = parse_motor_state_int16(rcv_data)
            if state is not None:
                state.id = motor_id
                return state
        return None

    def set_position(self, motor_id: int, pos_turns: float) -> MotorState | None:
        """位置控制 (单位: 圈)。注意：最大速度最大力矩，运动激烈，推荐用 set_pos_vel_acc。"""
        can_id = 0x8000 | motor_id
        pos_int = turns_to_int16(pos_turns)
        replies = self._send_can_and_recv(can_id, build_pos_int16(pos_int))
        for rcv_id, rcv_data in replies:
            state = parse_motor_state_int16(rcv_data)
            if state is not None:
                state.id = motor_id
                return state
        return None

    def set_velocity(self, motor_id: int, vel_rps: float) -> MotorState | None:
        """速度控制 (单位: 转/秒)。"""
        can_id = 0x8000 | motor_id
        vel_int = rps_to_int16(vel_rps)
        replies = self._send_can_and_recv(can_id, build_vel_int16(vel_int))
        for rcv_id, rcv_data in replies:
            state = parse_motor_state_int16(rcv_data)
            if state is not None:
                state.id = motor_id
                return state
        return None

    def set_pos_vel_tqe(self, motor_id: int, pos_turns: float, vel_rps: float, tqe_raw: int) -> MotorState | None:
        """位置+速度+最大力矩控制。"""
        can_id = 0x8000 | motor_id
        replies = self._send_can_and_recv(can_id, build_pos_vel_tqe_int16(
            turns_to_int16(pos_turns), rps_to_int16(vel_rps), tqe_raw))
        for rcv_id, rcv_data in replies:
            state = parse_motor_state_int16(rcv_data)
            if state is not None:
                state.id = motor_id
                return state
        return None

    def set_pos_vel_acc(self, motor_id: int, pos_turns: float, vel_max_rps: float, acc_rpss: float) -> MotorState | None:
        """梯形控制: 位置+最大速度+加速度 (推荐的位置控制方式)。
        pos: 圈, vel_max: 转/秒, acc: 转/秒^2
        """
        can_id = 0x8000 | motor_id
        acc_int = max(-32768, min(32767, int(acc_rpss / 0.001)))
        replies = self._send_can_and_recv(can_id, build_pos_velmax_acc_int16(
            turns_to_int16(pos_turns), rps_to_int16(vel_max_rps), acc_int))
        for rcv_id, rcv_data in replies:
            state = parse_motor_state_int16(rcv_data)
            if state is not None:
                state.id = motor_id
                return state
        return None

    def set_torque(self, motor_id: int, tqe_nm: float) -> MotorState | None:
        """力矩控制。tqe_nm: 力矩 (Nm)。"""
        can_id = 0x8000 | motor_id
        coeff = TORQUE_COEFF.get(motor_id, 1.0)
        tqe_int = max(-32768, min(32767, int(tqe_nm / coeff))) if coeff != 0 else 0
        replies = self._send_can_and_recv(can_id, build_torque_int16(tqe_int))
        for rcv_id, rcv_data in replies:
            state = parse_motor_state_int16(rcv_data)
            if state is not None:
                state.id = motor_id
                return state
        return None

    def set_voltage(self, motor_id: int, voltage_v: float) -> MotorState | None:
        """DQ 电压控制。voltage_v: 电压 (V)。"""
        can_id = 0x8000 | motor_id
        volt_int = max(-32768, min(32767, int(voltage_v / 0.1)))
        replies = self._send_can_and_recv(can_id, build_voltage_int16(volt_int))
        for rcv_id, rcv_data in replies:
            state = parse_motor_state_int16(rcv_data)
            if state is not None:
                state.id = motor_id
                return state
        return None

    def set_current(self, motor_id: int, current_a: float) -> MotorState | None:
        """DQ 电流控制。current_a: 电流 (A)。"""
        can_id = 0x8000 | motor_id
        cur_int = max(-32768, min(32767, int(current_a / 0.1)))
        replies = self._send_can_and_recv(can_id, build_current_int16(cur_int))
        for rcv_id, rcv_data in replies:
            state = parse_motor_state_int16(rcv_data)
            if state is not None:
                state.id = motor_id
                return state
        return None

    def set_pos_vel_tqe_kp_kd(self, motor_id: int,
                               pos_turns: float, vel_rps: float, tqe_nm: float,
                               kp: float, kd: float) -> MotorState | None:
        """运控模式: 输出力矩 = (pos偏差)*kp + (vel偏差)*kd + tqe。
        pos: 圈, vel: 转/秒, tqe: Nm, kp/kd: 增益。
        """
        can_id = 0x8000 | motor_id
        coeff = TORQUE_COEFF.get(motor_id, 1.0)
        tqe_int = max(-32768, min(32767, int(tqe_nm / coeff))) if coeff != 0 else 0
        kp_int = max(0, min(32767, int(kp)))
        kd_int = max(0, min(32767, int(kd * 100)))
        replies = self._send_can_and_recv(can_id, build_pos_vel_tqe_kp_kd_int16(
            turns_to_int16(pos_turns), rps_to_int16(vel_rps), tqe_int, kp_int, kd_int))
        for rcv_id, rcv_data in replies:
            state = parse_motor_state_int16(rcv_data)
            if state is not None:
                state.id = motor_id
                return state
        return None

    def reset_zero(self, motor_id: int) -> str:
        """重设电机零位。"""
        can_id = 0x8000 | motor_id
        return self._send_can(can_id, build_set_zero())

    def save_config(self, motor_id: int) -> str:
        """保存电机设置。"""
        can_id = 0x8000 | motor_id
        return self._send_can(can_id, build_conf_write())

    def motor_reset(self, motor_id: int) -> str:
        """电机软重启。"""
        can_id = 0x8000 | motor_id
        return self._send_can(can_id, build_motor_reset())

    def read_version(self, motor_id: int) -> str | None:
        """查询电机固件版本，返回版本字符串或 None。"""
        can_id = 0x8000 | motor_id
        replies = self._send_can_and_recv(can_id, build_read_version())
        for rcv_id, rcv_data in replies:
            if len(rcv_data) >= 4 and (rcv_data[0] & 0xF0) == 0x20 and rcv_data[1] == 0xB5:
                # 跳过 cmd + addr 后读取版本字节
                ver_bytes = rcv_data[2:]
                # 尝试 COMBINE_VERSION 格式: (major<<12)|(minor<<4)|patch
                if len(ver_bytes) >= 2:
                    ver16 = struct.unpack_from("<H", ver_bytes)[0]
                    major = (ver16 >> 12) & 0xF
                    minor = (ver16 >> 4) & 0xFF
                    patch = ver16 & 0xF
                    return f"v{major}.{minor}.{patch}"
        return None

    def set_timeout(self, motor_id: int, timeout_ms: int) -> str:
        """设置电机超时时间 (ms, 0=禁用)。"""
        can_id = 0x8000 | motor_id
        return self._send_can(can_id, build_set_timeout_int16(timeout_ms))
