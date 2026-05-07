// =============================================================================
//  bindings.cpp
//  pybind11 绑定: 把 motor_example_debug 的 C++ API 暴露给 Python
//
//  生成的 Python 模块: panthera_motor (import panthera_motor)
//
//  覆盖的 C++ 接口:
//    - 枚举: PosUnit, CanFault
//    - 结构体: MotorState, CanStatus, PortInfo, Stats, RobotConfig, ManyMotorCmd
//    - 自由函数: list_serial_ports, find_likely_debug_boards,
//                to_turns / from_turns, parse_motor_state_int16
//    - 类 HightorqueSerial: 全部 public 方法
// =============================================================================

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>          // std::vector / std::map / std::optional / std::pair
#include <pybind11/functional.h>   // std::function

#include "hightorque_serial.hpp"
#include "robot_config.hpp"

#ifdef _WIN32
  #ifndef NOMINMAX
  #define NOMINMAX
  #endif
  #include <windows.h>
#endif

#include <string>

namespace py = pybind11;
using namespace hightorque;

// ----------------------------------------------------------------------------
//  把"任意编码"的 std::string 安全转成 UTF-8 std::string.
//  Windows 下系统调用 (setup API) 返回的是 ACP (中文系统就是 GBK), 直接
//  当 UTF-8 给 pybind11 会抛 UnicodeDecodeError. 这里先 ACP -> UTF-16 -> UTF-8.
//  其他平台直接原样返回.
// ----------------------------------------------------------------------------
static std::string to_utf8_safe(const std::string& s) {
#ifdef _WIN32
    if (s.empty()) return s;
    // 1) 尝试当 UTF-8 验证: MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS)
    int wn = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS,
                                 s.data(), (int)s.size(), nullptr, 0);
    if (wn > 0) return s;   // 已是合法 UTF-8

    // 2) 当 ACP (中文系统通常是 936 GBK) -> UTF-16 -> UTF-8
    int n16 = MultiByteToWideChar(CP_ACP, 0, s.data(), (int)s.size(), nullptr, 0);
    if (n16 <= 0) return std::string();
    std::wstring w(n16, L'\0');
    MultiByteToWideChar(CP_ACP, 0, s.data(), (int)s.size(), &w[0], n16);

    int n8 = WideCharToMultiByte(CP_UTF8, 0, w.data(), n16, nullptr, 0, nullptr, nullptr);
    if (n8 <= 0) return std::string();
    std::string out(n8, '\0');
    WideCharToMultiByte(CP_UTF8, 0, w.data(), n16, &out[0], n8, nullptr, nullptr);
    return out;
#else
    return s;
#endif
}

// 把 PortInfo 里所有可能含中文的字段做一次 to_utf8_safe
static PortInfo sanitize_port_info(PortInfo p) {
    p.description = to_utf8_safe(p.description);
    p.hardware_id = to_utf8_safe(p.hardware_id);
    return p;
}

PYBIND11_MODULE(panthera_motor, m) {
    m.doc() = "Panthera-HT 调试板 Python 绑定 (pybind11 over motor_example_debug C++)";

    // ----------------------------------------------------------------------
    //  枚举
    // ----------------------------------------------------------------------
    py::enum_<PosUnit>(m, "PosUnit", "位置单位制")
        .value("Turns",   PosUnit::Turns,   "圈 (协议原生)")
        .value("Radians", PosUnit::Radians, "弧度")
        .value("Degrees", PosUnit::Degrees, "角度")
        .export_values();

    py::enum_<CanFault>(m, "CanFault", "CAN 总线状态")
        .value("Unknown",      CanFault::Unknown)
        .value("Ok",           CanFault::Ok)
        .value("ErrorWarning", CanFault::ErrorWarning)
        .value("ErrorPassive", CanFault::ErrorPassive)
        .value("BusOff",       CanFault::BusOff)
        .export_values();

    // ----------------------------------------------------------------------
    //  单位换算 + 工具函数
    // ----------------------------------------------------------------------
    m.def("to_turns",   &to_turns,   py::arg("value"), py::arg("unit"),
          "把任意单位的位置值换算成 '圈' (协议原生单位)");
    m.def("from_turns", &from_turns, py::arg("turns"), py::arg("unit"),
          "把 '圈' 换算成指定单位");

    // ----------------------------------------------------------------------
    //  结构体
    // ----------------------------------------------------------------------
    py::class_<MotorState>(m, "MotorState", "电机状态")
        .def(py::init<>())
        .def_readwrite("id",             &MotorState::id)
        .def_readwrite("mode",           &MotorState::mode)
        .def_readwrite("fault",          &MotorState::fault)
        .def_readwrite("position",       &MotorState::position, "位置 (圈)")
        .def_readwrite("velocity",       &MotorState::velocity, "速度 (转/秒)")
        .def_readwrite("torque",         &MotorState::torque,   "力矩 (raw int16)")
        .def_readwrite("pos_limit_flag", &MotorState::pos_limit_flag,
                       "软限位标志: 0=未触发, +1=超上限, -1=超下限")
        .def("to_string", &MotorState::to_string)
        .def("__repr__", [](const MotorState& s) {
            return "<MotorState id=" + std::to_string(s.id) +
                   " pos=" + std::to_string(s.position) +
                   " vel=" + std::to_string(s.velocity) +
                   " mode=" + std::to_string(s.mode) + ">";
        });

    py::class_<CanStatus>(m, "CanStatus", "CAN 总线状态结构")
        .def(py::init<>())
        .def_readwrite("fault",        &CanStatus::fault)
        .def_readwrite("lec",          &CanStatus::lec)
        .def_readwrite("tx_err_count", &CanStatus::tx_err_count)
        .def_readwrite("rx_err_count", &CanStatus::rx_err_count)
        .def_readwrite("raw",          &CanStatus::raw)
        .def("is_ok",     &CanStatus::is_ok)
        .def("to_string", &CanStatus::to_string)
        .def("__repr__", [](const CanStatus& s) { return "<CanStatus " + s.to_string() + ">"; });

    py::class_<PortInfo>(m, "PortInfo", "串口信息")
        .def(py::init<>())
        .def_readwrite("port",        &PortInfo::port)
        .def_readwrite("description", &PortInfo::description)
        .def_readwrite("hardware_id", &PortInfo::hardware_id)
        .def_readwrite("vid",         &PortInfo::vid)
        .def_readwrite("pid",         &PortInfo::pid)
        .def("__repr__", [](const PortInfo& p) {
            return "<PortInfo " + p.port + " VID=" + p.vid + " PID=" + p.pid + " " + p.description + ">";
        });

    py::class_<Stats>(m, "Stats", "收发统计")
        .def(py::init<>())
        .def_readwrite("tx_frames",        &Stats::tx_frames)
        .def_readwrite("rx_frames",        &Stats::rx_frames)
        .def_readwrite("rx_parsed",        &Stats::rx_parsed)
        .def_readwrite("rx_dropped",       &Stats::rx_dropped)
        .def_readwrite("last_rx_age_ms",   &Stats::last_rx_age_ms)
        .def_readwrite("avg_tx_period_ms", &Stats::avg_tx_period_ms)
        .def_readwrite("max_tx_jitter_ms", &Stats::max_tx_jitter_ms)
        .def("to_string", &Stats::to_string);

    // 串口枚举 (description/hardware_id 在中文 Windows 下是 GBK,
    // 这里 wrap 一层把它转成 UTF-8 再交给 pybind11)
    m.def("list_serial_ports", []() {
        auto ports = list_serial_ports();
        for (auto& p : ports) p = sanitize_port_info(std::move(p));
        return ports;
    }, "列出系统全部串口 (description/hardware_id 已转 UTF-8)");

    m.def("find_likely_debug_boards",
          [](const std::vector<std::string>& known_vids) {
              auto ports = find_likely_debug_boards(known_vids);
              for (auto& p : ports) p = sanitize_port_info(std::move(p));
              return ports;
          },
          py::arg("known_vids") = std::vector<std::string>{"0483","0403","1A86","10C4","067B"},
          "按常见 USB-Serial 调试板 VID 过滤候选");
    m.def("parse_motor_state_int16", &parse_motor_state_int16,
          py::arg("can_data"), "解析电机回复的 int16 状态帧");

    // ----------------------------------------------------------------------
    //  RobotConfig
    // ----------------------------------------------------------------------
    py::class_<RobotConfig>(m, "RobotConfig", "INI 风格的机器人配置")
        .def(py::init<>())
        .def_readwrite("port",            &RobotConfig::port)
        .def_readwrite("baudrate",        &RobotConfig::baudrate)
        .def_readwrite("motor_ids",       &RobotConfig::motor_ids)
        .def_readwrite("pos_unit",        &RobotConfig::pos_unit)
        .def_readwrite("limits",          &RobotConfig::limits,
                       "字典: motor_id -> (lo, hi) (圈)")
        .def_readwrite("control_rate_hz", &RobotConfig::control_rate_hz)
        .def_readwrite("max_torque_raw",  &RobotConfig::max_torque_raw)
        .def_readwrite("use_async_rx",    &RobotConfig::use_async_rx)
        .def_readwrite("trajectory_dt_s", &RobotConfig::trajectory_dt_s)
        .def("find_limit",      &RobotConfig::find_limit,      py::arg("motor_id"))
        .def("apply_limits_to", &RobotConfig::apply_limits_to, py::arg("ht"))
        .def("to_string",       &RobotConfig::to_string)
        .def_static("load",     &RobotConfig::load,            py::arg("path"),
                    "从文件加载 (失败抛 RuntimeError)")
        .def("__repr__", [](const RobotConfig& c) { return "<RobotConfig\n" + c.to_string() + "\n>"; });

    // ----------------------------------------------------------------------
    //  HightorqueSerial::ManyMotorCmd 嵌套类型
    // ----------------------------------------------------------------------
    py::class_<HightorqueSerial::ManyMotorCmd>(m, "ManyMotorCmd",
        "一拖多模式下的单电机指令")
        .def(py::init<>())
        .def(py::init([](int motor_id, double pos, double vel_rps, int tqe_raw) {
            HightorqueSerial::ManyMotorCmd c;
            c.motor_id = motor_id;
            c.pos      = pos;
            c.vel_rps  = vel_rps;
            c.tqe_raw  = tqe_raw;
            return c;
        }), py::arg("motor_id"), py::arg("pos"), py::arg("vel_rps"), py::arg("tqe_raw") = 0)
        .def_readwrite("motor_id", &HightorqueSerial::ManyMotorCmd::motor_id)
        .def_readwrite("pos",      &HightorqueSerial::ManyMotorCmd::pos)
        .def_readwrite("vel_rps",  &HightorqueSerial::ManyMotorCmd::vel_rps)
        .def_readwrite("tqe_raw",  &HightorqueSerial::ManyMotorCmd::tqe_raw);

    // ----------------------------------------------------------------------
    //  HightorqueSerial 主类
    //  关键: 释放 GIL 让 Web 线程能并发查询其它接口 (尤其 Flask 多请求时)
    // ----------------------------------------------------------------------
    py::class_<HightorqueSerial>(m, "HightorqueSerial",
        "调试板 USB 串口驱动 (CAN-FD 透传)")
        .def(py::init<const std::string&, uint32_t>(),
             py::arg("port"), py::arg("baudrate") = 4'000'000u,
             "打开串口, 失败抛 RuntimeError")

        // -- 基础 --
        .def("close",   &HightorqueSerial::close,   py::call_guard<py::gil_scoped_release>())
        .def("is_open", &HightorqueSerial::is_open)

        // -- 异步 RX --
        .def("enable_async_rx",  &HightorqueSerial::enable_async_rx,
             py::call_guard<py::gil_scoped_release>())
        .def("disable_async_rx", &HightorqueSerial::disable_async_rx,
             py::call_guard<py::gil_scoped_release>())
        .def("is_async_rx",      &HightorqueSerial::is_async_rx)

        .def("get_state",  &HightorqueSerial::get_state,  py::arg("motor_id"))
        .def("get_states", &HightorqueSerial::get_states, py::arg("motor_ids"))
        .def("wait_state", &HightorqueSerial::wait_state,
             py::arg("motor_id"), py::arg("timeout_s") = 0.1,
             py::call_guard<py::gil_scoped_release>())

        .def("get_stats",   &HightorqueSerial::get_stats)
        .def("reset_stats", &HightorqueSerial::reset_stats)

        // -- 调试板命令 --
        .def("can_status",      &HightorqueSerial::can_status,
             py::call_guard<py::gil_scoped_release>())
        .def("read_can_status", &HightorqueSerial::read_can_status,
             py::call_guard<py::gil_scoped_release>())
        .def("can_config",      &HightorqueSerial::can_config,
             py::call_guard<py::gil_scoped_release>())

        // -- 软限位 --
        .def("enable_position_limit",     &HightorqueSerial::enable_position_limit,
             py::arg("motor_id"), py::arg("lo"), py::arg("hi"),
             py::arg("unit") = PosUnit::Turns)
        .def("disable_position_limit",    &HightorqueSerial::disable_position_limit,
             py::arg("motor_id"))
        .def("clear_all_position_limits", &HightorqueSerial::clear_all_position_limits)
        .def("get_position_limit_turns",
             [](const HightorqueSerial& self, int motor_id)
                 -> std::optional<std::pair<double, double>> {
                 double lo = 0.0, hi = 0.0;
                 if (self.get_position_limit_turns(motor_id, lo, hi))
                     return std::make_pair(lo, hi);
                 return std::nullopt;
             }, py::arg("motor_id"),
             "返回 (lo, hi) 圈; 未设限位时返回 None")

        // -- 电机控制 (释放 GIL: 串口 IO 是慢操作) --
        .def("read_motor_state", &HightorqueSerial::read_motor_state,
             py::arg("motor_id"), py::arg("timeout_s") = 0.5,
             py::call_guard<py::gil_scoped_release>())
        .def("stop",             &HightorqueSerial::stop,  py::arg("motor_id"),
             py::call_guard<py::gil_scoped_release>())
        .def("brake",            &HightorqueSerial::brake, py::arg("motor_id"),
             py::call_guard<py::gil_scoped_release>())

        .def("set_position",     &HightorqueSerial::set_position,
             py::arg("motor_id"), py::arg("pos"), py::arg("unit") = PosUnit::Turns,
             py::call_guard<py::gil_scoped_release>())
        .def("set_velocity",     &HightorqueSerial::set_velocity,
             py::arg("motor_id"), py::arg("vel_rps"),
             py::call_guard<py::gil_scoped_release>())
        .def("set_pos_vel_tqe",  &HightorqueSerial::set_pos_vel_tqe,
             py::arg("motor_id"), py::arg("pos"), py::arg("vel_rps"), py::arg("tqe_raw"),
             py::arg("unit") = PosUnit::Turns,
             py::call_guard<py::gil_scoped_release>())
        .def("set_pos_vel_acc",  &HightorqueSerial::set_pos_vel_acc,
             py::arg("motor_id"), py::arg("pos"), py::arg("vel_max_rps"), py::arg("acc_rpss"),
             py::arg("unit") = PosUnit::Turns,
             py::call_guard<py::gil_scoped_release>())

        .def("set_many_pos_vel_tqe", &HightorqueSerial::set_many_pos_vel_tqe,
             py::arg("cmds"), py::arg("pos_unit") = PosUnit::Turns,
             py::arg("max_motor_id") = 0, py::arg("timeout_s") = 0.05,
             py::call_guard<py::gil_scoped_release>())

        .def("set_torque",  &HightorqueSerial::set_torque,
             py::arg("motor_id"), py::arg("tqe_nm"), py::arg("motor_model") = "",
             py::call_guard<py::gil_scoped_release>())
        .def("set_voltage", &HightorqueSerial::set_voltage,
             py::arg("motor_id"), py::arg("voltage_v"),
             py::call_guard<py::gil_scoped_release>())
        .def("set_current", &HightorqueSerial::set_current,
             py::arg("motor_id"), py::arg("current_a"),
             py::call_guard<py::gil_scoped_release>())

        .def("set_pos_vel_tqe_kp_kd", &HightorqueSerial::set_pos_vel_tqe_kp_kd,
             py::arg("motor_id"), py::arg("pos"), py::arg("vel_rps"),
             py::arg("tqe_nm"), py::arg("kp"), py::arg("kd"),
             py::arg("motor_model") = "", py::arg("unit") = PosUnit::Turns,
             py::call_guard<py::gil_scoped_release>())

        // -- 配置/版本 --
        .def("reset_zero",  &HightorqueSerial::reset_zero,  py::arg("motor_id"),
             py::call_guard<py::gil_scoped_release>())
        .def("save_config", &HightorqueSerial::save_config, py::arg("motor_id"),
             py::call_guard<py::gil_scoped_release>())
        .def("motor_reset", &HightorqueSerial::motor_reset, py::arg("motor_id"),
             py::call_guard<py::gil_scoped_release>())
        .def("read_version", &HightorqueSerial::read_version, py::arg("motor_id"),
             py::call_guard<py::gil_scoped_release>())
        .def("set_timeout", &HightorqueSerial::set_timeout,
             py::arg("motor_id"), py::arg("timeout_ms"),
             py::call_guard<py::gil_scoped_release>())

        // -- 后台轮询 --
        .def("start_state_polling", &HightorqueSerial::start_state_polling,
             py::arg("motor_ids"), py::arg("rate_hz") = 50.0,
             py::arg("on_update") = std::function<void(const std::vector<int>&)>{},
             py::call_guard<py::gil_scoped_release>())
        .def("stop_state_polling",  &HightorqueSerial::stop_state_polling,
             py::call_guard<py::gil_scoped_release>())
        .def("is_polling",          &HightorqueSerial::is_polling)
        .def("get_cached_state",    &HightorqueSerial::get_cached_state, py::arg("motor_id"));
}
