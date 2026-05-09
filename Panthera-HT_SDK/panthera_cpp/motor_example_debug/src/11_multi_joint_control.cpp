// =============================================================================
//  11_multi_joint_control.cpp
//  多关节交互式位置控制 (七自由度机械臂) — 250Hz 异步 + 一拖多 + S 曲线版
//
//  本版相对前一版的核心升级:
//    1. 内核切到 HightorqueSerial::run_control_loop + 一拖多 (CAN 0x8090)
//       - 单 tick: 1 帧 set_many_pos_vel_tqe(7) + 读 cache, ~1-3 ms (async fire-and-forget)
//       - 7 个关节命令在同一 CAN-FD 帧, 时间一致性 100%
//       - 默认 250 Hz 控制频率 (周期 4ms; 200~300 Hz 推荐, 500 Hz 可调)
//    2. 后台 RX 线程持续接收所有反馈, 主线程只读 cache, 不阻塞
//    3. 上层 S 曲线轨迹规划 (cosine ease-in-out): 起停柔, 中段快
//         pos(α)  = start + 0.5·(1 - cos(π·α)) · (target - start)
//         vel(α)  = (π/2)·v_avg · sin(π·α)         ← 与 pos 导数严格一致
//         a_peak  = (π/2)² / dt  · |Δpos|         ← 比线性插值平稳, 无加速度阶跃
//       优点: 起步/到位都自然减速到 0, 解决了线性插值"突然启停 + 卡顿"的问题
//    4. 配置全部走 robot.cfg: control_rate_hz / trajectory_dt_s / max_torque_raw
//    5. 软限位仍 100% 走驱动层 (HightorqueSerial::enable_position_limit)
//
//  用法:
//      11_multi_joint_control.exe                    # 默认读同目录的 robot.cfg
//      11_multi_joint_control.exe path/to/your.cfg   # 指定配置文件
//
//  机械臂场景安全提醒:
//    - 首次运行先用单关节脚本确认各关节正常
//    - 肩关节(2号)和肘关节(3号)承重, stop 后保持力矩
//    - 4~7 号限位为保守初值, 务必按实际行程在 robot.cfg 中修改
//    - 如电机失控, 直接拔掉调试板 USB 线
//    - 运动中 Ctrl+Q 紧急停止 (会断驱动+回 stop)
// =============================================================================

#include "hightorque_serial.hpp"
#include "robot_config.hpp"
#include "port_picker.hpp"     // hightorque::pick_serial_port() — USB 自动识别

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
  #include <conio.h>
  #include <io.h>
  #ifndef NOMINMAX
  #define NOMINMAX
  #endif
  #include <windows.h>
#else
  #include <fcntl.h>
  #include <sys/select.h>
  #include <termios.h>
  #include <unistd.h>
#endif

using hightorque::HightorqueSerial;
using hightorque::MotorState;
using hightorque::RobotConfig;
using hightorque::PosUnit;

// ---------------------------------------------------------------------------
//  运动参数 (上层 S 曲线插值用, 与电机内梯形控制无关)
//
//  设计原则: vel 命令必须严格等于 desired 位置的瞬时导数, 否则电机内 mode=10
//          PID 会出现 "vel 比 desired 变化率快很多 → 一冲就到 desired → 停下
//          → 等 desired 爬 → 再冲" 的 stop-and-go 振荡.
//
//  ⇒ 不再用 V_PEAK_MIN 抬高 vel ! 改为自适应 dt: 动作太小时整段时长缩短.
//
//  - VEL_AVG_MAX:    单段轨迹的平均速度上限. S 曲线峰值 = π/2 · 平均.
//  - V_AVG_TARGET:   期望平均速度. 当 |Δpos| / 用户指定 dt < 此值时, 自动缩短 dt 到
//                    |Δpos| / V_AVG_TARGET, 让 v_avg 至少有这么大. 0.05 rps 约 18°/s,
//                    高于电机控制死区, 电机能完美跟踪.
//  - DT_MIN:         自适应下限. 单段不少于这么久, 防超快.
//  - SETTLE_TICKS_MS: 到位后 hold 多少 ms 让 cache 收敛.
// ---------------------------------------------------------------------------

static constexpr double PI               = 3.14159265358979323846;
static constexpr double VEL_AVG_MAX      = 0.5;    // 转/秒 (单关节平均速度上限, 内部 rps)
static constexpr double V_AVG_TARGET     = 0.05;   // 转/秒 (自适应目标平均速度, 内部 rps)
static constexpr double DT_MIN           = 0.3;    // 秒 (自适应 dt 的下限)
static constexpr int    SETTLE_TICKS_MS  = 300;    // 到位后 hold 时长 (ms)

// 用户接口单位换算: 1 圈 = 360 度
//   内部所有数据流 (start_pos / targets / cmds / limits / read_motor_state) 都按"圈" (协议原生).
//   只在用户输入解析后立刻转 turns, 显示前再转 deg, 不污染中间逻辑.
static constexpr double DEG_PER_TURN     = 360.0;
inline double deg_to_turns(double d) { return d / DEG_PER_TURN; }
inline double turns_to_deg(double t) { return t * DEG_PER_TURN; }

// ---------------------------------------------------------------------------
//  紧急停止异常 (由 Ctrl+Q 触发)
// ---------------------------------------------------------------------------

struct EmergencyStop : public std::exception {
    const char* what() const noexcept override { return "EmergencyStop"; }
};

// ---------------------------------------------------------------------------
//  Ctrl+Q (ASCII 0x11) 检测 + 自定义行输入  (与旧版完全相同, 略...)
// ---------------------------------------------------------------------------

#ifndef _WIN32
namespace {
struct TermiosRaw {
    int fd;
    termios old{};
    bool active = false;

    explicit TermiosRaw(int fd_) : fd(fd_) {
        if (tcgetattr(fd, &old) == 0) {
            termios raw = old;
            cfmakeraw(&raw);
            if (tcsetattr(fd, TCSADRAIN, &raw) == 0) active = true;
        }
    }
    ~TermiosRaw() {
        if (active) tcsetattr(fd, TCSADRAIN, &old);
    }
};
} // namespace
#endif

// 紧急停止键: 优先 ESC (0x1B), 兼容 Ctrl+Q (0x11).
//   - Windows console 默认会拦截 Ctrl+Q 当 XON 流控字符, 不一定能传到 _getch.
//   - ESC 永远直传, 最稳.
static bool poll_abort_key() {
#ifdef _WIN32
    while (_kbhit()) {
        const int ch = _getch();
        if (ch == 0x1B || ch == 0x11) return true;     // ESC or Ctrl+Q
        if (ch == 0xE0 || ch == 0x00) {
            if (_kbhit()) _getch();
        }
    }
    return false;
#else
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(STDIN_FILENO, &rfds);
    timeval tv{0, 0};
    if (select(STDIN_FILENO + 1, &rfds, nullptr, nullptr, &tv) <= 0) return false;
    char ch = 0;
    if (read(STDIN_FILENO, &ch, 1) != 1) return false;
    return ch == 0x1B || ch == 0x11;
#endif
}

// 兼容旧名字
static inline bool poll_ctrl_q() { return poll_abort_key(); }

static void sleep_abortable(double seconds) {
    using namespace std::chrono;
    const auto end = steady_clock::now() + microseconds(static_cast<long long>(seconds * 1e6));
    while (true) {
        if (poll_ctrl_q()) throw EmergencyStop();
        const auto now = steady_clock::now();
        if (now >= end) return;
        const auto remain = duration_cast<microseconds>(end - now).count();
        std::this_thread::sleep_for(microseconds(std::min<long long>(50'000, remain)));
    }
}

static std::string input_line(const std::string& prompt) {
    std::cout << prompt << std::flush;

#ifdef _WIN32
    std::string buf;
    while (true) {
        const int ch = _getch();
        if (ch == 0x1B || ch == 0x11) throw EmergencyStop();   // ESC or Ctrl+Q
        if (ch == '\r' || ch == '\n') {
            std::cout << '\n' << std::flush;
            // 吃掉紧跟的 \r/\n (粘贴或 console 输入末尾带 \r\n, 否则下次会立刻 return "")
            // 短暂 sleep 让 console buffer 里的 \n 真的就位再 _kbhit
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            while (_kbhit()) {
                const int peek = _getch();
                if (peek != '\r' && peek != '\n') {
                    // 不是行结束符 — 这是用户的下一次输入开头, 没法塞回
                    // 这种情况罕见 (粘贴 "home\rXXX" 才会发生), 不处理
                    break;
                }
            }
            return buf;
        }
        if (ch == 0x08 || ch == 0x7F) {
            if (!buf.empty()) {
                buf.pop_back();
                std::cout << "\b \b" << std::flush;
            }
            continue;
        }
        if (ch == 0xE0 || ch == 0x00) {
            if (_kbhit()) _getch();
            continue;
        }
        buf.push_back(static_cast<char>(ch));
        std::cout << static_cast<char>(ch) << std::flush;
    }
#else
    TermiosRaw raw(STDIN_FILENO);
    std::string buf;
    while (true) {
        char ch = 0;
        const ssize_t n = read(STDIN_FILENO, &ch, 1);
        if (n <= 0) continue;
        if (ch == 0x1B || ch == 0x11) throw EmergencyStop();   // ESC or Ctrl+Q
        if (ch == '\r' || ch == '\n') {
            std::cout << "\r\n" << std::flush;
            return buf;
        }
        if (ch == 0x7F || ch == 0x08) {
            if (!buf.empty()) {
                buf.pop_back();
                std::cout << "\b \b" << std::flush;
            }
            continue;
        }
        if (static_cast<unsigned char>(ch) >= 32) {
            buf.push_back(ch);
            std::cout << ch << std::flush;
        }
    }
#endif
}

// ---------------------------------------------------------------------------
//  字符串工具
// ---------------------------------------------------------------------------

static std::string strip_lower(const std::string& s) {
    auto b = s.begin();
    auto e = s.end();
    while (b != e && std::isspace(static_cast<unsigned char>(*b))) ++b;
    while (e != b && std::isspace(static_cast<unsigned char>(*(e - 1)))) --e;
    std::string out(b, e);
    std::transform(out.begin(), out.end(), out.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    return out;
}

static std::vector<std::string> split_ws(const std::string& s) {
    std::vector<std::string> out;
    std::istringstream iss(s);
    for (std::string tok; iss >> tok;) out.push_back(std::move(tok));
    return out;
}

static bool starts_with(const std::string& s, const std::string& p) {
    return s.size() >= p.size() && std::equal(p.begin(), p.end(), s.begin());
}

static bool parse_double(const std::string& s, double& out) {
    try {
        std::size_t idx = 0;
        out = std::stod(s, &idx);
        if (idx != s.size()) return false;
        return true;
    } catch (...) {
        return false;
    }
}

static bool parse_int(const std::string& s, int& out) {
    try {
        std::size_t idx = 0;
        out = std::stoi(s, &idx);
        if (idx != s.size()) return false;
        return true;
    } catch (...) {
        return false;
    }
}

// ---------------------------------------------------------------------------
//  限位预检 (UI 层): 不修改值, 只在屏幕上提示用户"超界, 驱动会自动 clamp"
//  实际 clamp 由驱动层 set_pos_vel_acc 负责.
// ---------------------------------------------------------------------------

static void warn_if_out_of_limit(const HightorqueSerial& ht, int motor_id, double pos_deg) {
    const double pos_turns = deg_to_turns(pos_deg);
    double lo = 0.0, hi = 0.0;
    if (!ht.get_position_limit_turns(motor_id, lo, hi)) return;     // 无限位, 不预警
    if (pos_turns >= lo && pos_turns <= hi) return;
    const double clamped_turns = (pos_turns > hi) ? hi : lo;
    std::cout << "    ! 电机 " << motor_id << ": "
              << std::fixed << std::setprecision(2) << pos_deg
              << " 度 超出限位 [" << turns_to_deg(lo) << ", " << turns_to_deg(hi)
              << "] 度, 将由驱动自动截断为 " << turns_to_deg(clamped_turns) << " 度\n";
}

// ---------------------------------------------------------------------------
//  状态读取
//
//  关键点: async RX 是"被动接收", 电机不会主动报告状态, 只在收到带查询子帧的
//  控制命令后才回. 所以"读状态"必须主动发一次查询触发 cache 刷新, 而不能直接
//  读 cache (cache 可能是空的或者老的).
//
//  在 async 模式下, read_motor_state(id) 内部会:
//    1) 发单电机查询帧 (CAN ID = 0x8000|id, payload = build_read_state_int16)
//    2) 用条件变量等 cache 里这个 id 的 seq 自增
//    3) 返回最新 cache
// ---------------------------------------------------------------------------

static void read_all(HightorqueSerial& ht, const std::vector<int>& motor_ids) {
    std::cout << "\n  当前状态" << (ht.is_async_rx() ? " (async, 主动查询触发 cache 刷新):" : ":") << "\n";

    for (int mid : motor_ids) {
        // 主动查询; async 模式下走条件变量等 RX 线程更新, 同步模式下走原阻塞路径
        const auto s = ht.read_motor_state(mid, 0.05);
        if (s) {
            double lo_t = 0.0, hi_t = 0.0;
            const bool has_limit = ht.get_position_limit_turns(mid, lo_t, hi_t);
            std::cout << "    电机 " << mid << ": "
                      << "pos=" << std::showpos << std::fixed << std::setprecision(2)
                                << turns_to_deg(s->position) << std::noshowpos << " 度  "
                      << "vel=" << std::showpos << std::fixed << std::setprecision(2)
                                << turns_to_deg(s->velocity) << std::noshowpos << " 度/秒  "
                      << "mode=" << s->mode;
            if (has_limit) {
                std::cout << "  限位=[" << std::showpos << std::fixed << std::setprecision(1)
                          << turns_to_deg(lo_t) << ", " << turns_to_deg(hi_t)
                          << std::noshowpos << "] 度";
            } else {
                std::cout << "  限位=(未设)";
            }
            std::cout << "\n";
        } else {
            std::cout << "    电机 " << mid << ": 50ms 内无回包 (检查 USB / 电机供电)\n";
        }
    }
}

// ---------------------------------------------------------------------------
//  高频异步控制环 + 上层 S 曲线插值 (cosine ease-in-out)
//
//  每个 tick (默认 250Hz, 周期 4ms):
//    1) 算归一化进度 α = tick / total_ticks ∈ [0, 1]
//    2) 位置: pos(α) = start + 0.5·(1 - cos(π·α)) · (target - start)
//             两端导数 = 0 (软启动/软停止), 中段最快, 无加速度阶跃
//    3) 速度: vel(α) = v_peak · sin(π·α), 与 pos 导数严格匹配
//             v_peak  = (π/2) · (|Δpos| / dt_s)  [转/秒]
//             ⇒ 中段 vel 是 "平均 vel" 的 1.57 倍, 自动克服静摩擦
//             ⇒ 两端 vel = 0, 电机内 PID 顺势收敛, 不会"过冲后再回拉"
//    4) 一帧 set_many_pos_vel_tqe 同步广播 7 关节 (CAN 0x8090)
//    5) async 模式下 set_many fire-and-forget, RX 线程异步入 cache
//
//  到位后再发 SETTLE_TICKS_MS (默认 300ms) 持续 hold, 让 cache 收敛
//
//  退出处理: ESC/Ctrl+Q → run_control_loop 内自动 stop 所有电机 (mode=0)
//          正常完成 → 不 stop, 维持 mode=10 + 当前位置 hold (下次直接续控)
// ---------------------------------------------------------------------------

static void move_all_async(HightorqueSerial& ht,
                           const std::vector<int>& motor_ids,
                           const std::map<int, double>& targets,
                           const RobotConfig& cfg) {
    const double rate_hz = std::max(10.0, cfg.control_rate_hz);
    std::cout << "\n  运动中 (" << static_cast<int>(rate_hz) << "Hz 异步 + S 曲线, ESC 紧急停止)...\n";

    if (!ht.is_async_rx()) {
        std::cout << "  [警告] async RX 未启用, 临时降级为单帧同步发送 (会慢)\n";
    }

    // 1) 起点: 强制主动查询一次 (async 模式下 read_motor_state 会发查询 + 等 cache 更新)
    //    不能直接读 cache — 如果 cache 是空的或者是几秒前的旧值, start 错了电机会跳一大下.
    std::map<int, double> start_pos;
    for (int mid : motor_ids) {
        auto s = ht.read_motor_state(mid, 0.1);
        if (!s) {
            std::cout << "  [错误] 拿不到电机 " << mid << " 的当前位置, 放弃运动以保安全\n";
            return;
        }
        start_pos[mid] = s->position;
    }

    // 2) 自适应单段时长: 动作太小时缩短 dt, 让 v_avg 至少 V_AVG_TARGET.
    //    所有关节用同一个 dt 保证多关节同步到位.
    const double dt_user = std::max(DT_MIN, cfg.trajectory_dt_s);

    double max_abs_dpos = 0.0;
    for (int mid : motor_ids) {
        auto it = targets.find(mid);
        if (it == targets.end()) continue;
        max_abs_dpos = std::max(max_abs_dpos, std::abs(it->second - start_pos[mid]));
    }

    double dt_s = dt_user;
    bool   dt_adjusted = false;
    if (max_abs_dpos > 1e-5) {
        const double dt_target = max_abs_dpos / V_AVG_TARGET;
        if (dt_target < dt_user) {
            dt_s = std::max(DT_MIN, dt_target);
            dt_adjusted = true;
        }
    }
    if (dt_adjusted) {
        std::cout << "  [自适应 dt] 最大动作 " << std::fixed << std::setprecision(2)
                  << turns_to_deg(max_abs_dpos) << " 度, 段时长由 " << std::setprecision(2)
                  << cfg.trajectory_dt_s << "s 缩短为 " << dt_s
                  << "s (v_avg ≈ " << std::setprecision(2)
                  << turns_to_deg(max_abs_dpos / dt_s) << " 度/秒)\n";
    }

    // 3) 每电机的 (Δpos, v_peak_signed): 一次性算好, tick 内只查表
    //    - dpos 带符号 (target - start), 用于 desired 插值
    //    - v_peak_signed 严格等于 (π/2)·dpos/dt, vel 命令与 desired 变化率匹配
    //    - 不再加 V_PEAK_MIN: 自适应 dt 已保证 v_avg ≥ V_AVG_TARGET, 不会落入死区
    struct MotionPlan { double dpos; double v_peak_signed; };
    std::map<int, MotionPlan> plans;
    for (int mid : motor_ids) {
        auto it = targets.find(mid);
        if (it == targets.end()) continue;
        const double dpos = it->second - start_pos[mid];
        const double abs_dpos = std::abs(dpos);
        if (abs_dpos < 1e-5) {
            plans[mid] = {0.0, 0.0};                          // 已在目标位置
            continue;
        }
        const double v_avg  = abs_dpos / dt_s;
        const double v_peak = std::min(VEL_AVG_MAX, v_avg) * (PI / 2.0);
        plans[mid] = {dpos, std::copysign(v_peak, dpos)};
    }

    const int total_ticks   = std::max(1, static_cast<int>(dt_s * rate_hz));
    const int settle_ticks  = std::max(1, static_cast<int>(SETTLE_TICKS_MS * rate_hz / 1000.0));
    const int last_tick     = total_ticks + settle_ticks;
    bool      limit_reported = false;

    // 强制 set_many 每帧都包含全部电机的槽位, 见循环内详细注释
    const int max_mid = motor_ids.empty() ? 0
        : *std::max_element(motor_ids.begin(), motor_ids.end());

    HightorqueSerial::ControlLoopOptions opt;
    opt.rate_hz         = rate_hz;
    opt.stop_motor_ids  = motor_ids;
    opt.abort_check     = []() { return poll_ctrl_q(); };
    opt.on_exception    = [](const std::exception& e) {
        std::cout << "  [异常] " << e.what() << "\n";
    };

    // 进度打印间隔: ~0.4 秒一次 (与频率解耦, 不论 100/250/500Hz 都不刷屏)
    const int print_every_tick = std::max(1, static_cast<int>(rate_hz * 0.4));
    int last_print_tick = -print_every_tick;

    const int rc = ht.run_control_loop(opt, [&](int tick, double /*dt_ms*/) -> bool {
        if (tick >= last_tick) return false;

        // ---- S 曲线进度 ----
        // 主轨迹段: alpha 0→1; settle 段: alpha 钳到 1, sin(π) = 0 ⇒ vel=0 自然 hold
        const double alpha_raw = static_cast<double>(tick) / total_ticks;
        const double alpha     = std::min(1.0, alpha_raw);
        const double smooth    = 0.5 * (1.0 - std::cos(PI * alpha));   // 位置因子 [0..1]
        const double vel_factor = std::sin(PI * alpha);                // 速度因子 [0..1..0]

        // ---- 拼接一拖多命令 ----
        //
        // ⚠️ 设计 1: 每帧都给"全部 motor_ids"命令, 不在 targets 里的电机 hold
        //   (desired = start_pos, vel = 0). 不要把不在 cmds 里的电机交给 set_many
        //   内部填 NAN_INT16 (0x8000) — 部分 firmware 不识别 0x8000 为 NAN, 会按
        //   "vel = -32768 LSB = -8.19 rps" 解析, 让那个电机疯狂旋转.
        //   实测 (用户 log): 控制 M3 时 M4 飘到 -0.1344 圈, 就是 NAN_INT16 被误解为
        //   速度命令导致 M4 实际旋转 ≈ 8 rps × 控制时长 / 内部限速.
        //
        // ⚠️ 设计 2: cmds 里的 pos 必须是"圈" — start_pos 来自 read_motor_state (圈),
        //   targets 也已在 interactive_loop 解析时由"度"转成"圈"存入. set_many 必须
        //   传 PosUnit::Turns, 不能传 cfg.pos_unit (cfg 是用户接口单位偏好, 与协议层无关).
        //
        // ⚠️ 设计 3: max_motor_id 传 max_mid (= 配置中最大 ID), 让 set_many 永远发完整
        //   N 槽位帧, 末尾查询子帧 0x17 0x01 让所有 N 个电机回报 status, cache 始终新鲜.
        std::vector<HightorqueSerial::ManyMotorCmd> cmds;
        cmds.reserve(motor_ids.size());
        for (int mid : motor_ids) {
            auto pit = plans.find(mid);
            if (pit != plans.end() && pit->second.v_peak_signed != 0.0) {
                // 在 targets 里且有动作: 跟随 S 曲线
                const double desired = start_pos[mid] + smooth * pit->second.dpos;
                const double v_inst  = pit->second.v_peak_signed * vel_factor;
                cmds.push_back({mid, desired, v_inst, cfg.max_torque_raw});
            } else {
                // 不在 targets 里 / 已在目标位置: hold 在 start_pos
                cmds.push_back({mid, start_pos[mid], 0.0, cfg.max_torque_raw});
            }
        }
        auto states = ht.set_many_pos_vel_tqe(cmds, hightorque::PosUnit::Turns,
                                              /*max_motor_id=*/max_mid, /*timeout=*/0.002);

        // 限位检测 (async 下 set_many 返回的 cache 里带 pos_limit_flag)
        if (!limit_reported) {
            for (auto& [mid, st] : states) {
                if (st.pos_limit_flag != 0) {
                    std::cout << "    [限位] 电机 " << mid << " 目标超界, 驱动已截断 (flag="
                              << st.pos_limit_flag << ")\n";
                    limit_reported = true;
                    break;
                }
            }
        }

        // ---- 进度打印 ----
        if (tick - last_print_tick >= print_every_tick || tick == 0) {
            last_print_tick = tick;
            auto cur_states = ht.is_async_rx() ? ht.get_states(motor_ids) : states;
            std::ostringstream parts;
            bool first = true;
            for (int mid : motor_ids) {
                auto sit = cur_states.find(mid);
                auto tit = targets.find(mid);
                if (sit == cur_states.end() || tit == targets.end()) continue;
                const double err_deg = turns_to_deg(tit->second - sit->second.position);
                if (!first) parts << " | ";
                parts << "M" << mid
                      << "=" << std::showpos << std::fixed << std::setprecision(2)
                      << turns_to_deg(sit->second.position)
                      << "(err=" << err_deg << "°)" << std::noshowpos;
                first = false;
            }
            const auto stats = ht.get_stats();
            std::cout << "    [" << std::setw(4) << (tick + 1) << "/" << last_tick
                      << " α=" << std::fixed << std::setprecision(2) << alpha
                      << " v×=" << std::setprecision(2) << vel_factor
                      << "] " << parts.str()
                      << "  (jitter=" << std::fixed << std::setprecision(2)
                      << stats.max_tx_jitter_ms << "ms)\n";
        }
        return true;
    });

    if (rc == 1) throw EmergencyStop();
    if (rc == 2) std::cout << "  [警告] 控制环异常退出\n";

    const auto stats = ht.get_stats();
    std::cout << "  到位. " << stats.to_string() << "\n";
    read_all(ht, motor_ids);
}

// ---------------------------------------------------------------------------
//  通用模式切换 (启动时 + interactive 'release/brake/free' 命令复用)
//
//  mode 参数:
//    0x0A: 位置/速度/力矩控制 (set_motor_mode 内部自动 hold 当前位置, 不会跳)
//    0x0F: 刹车 (短路三相绕组提供阻尼, 但无主动力矩)
//    0x00: 停止 (断电, 电机可手动转动 — 适合调试时手动摆位姿)
//
//  返回 true = 全部电机切换成功.
//  失败原因常见: 上次会话留 fault, 需要电机断电 5 秒重启.
// ---------------------------------------------------------------------------

static bool switch_mode_all(HightorqueSerial& ht,
                            const std::vector<int>& motor_ids,
                            uint8_t mode, const std::string& mode_name,
                            int max_retry = 3) {
    std::cout << "  切换至 " << mode_name << " (mode=0x" << std::hex
              << std::setw(2) << std::setfill('0') << static_cast<int>(mode)
              << std::dec << std::setfill(' ') << ")...\n";

    // brake (0x0F) / stop (0x00) 在 async 模式下 set_motor_mode() 返回 nullopt
    // (control_call 在 async 下走 fire-and-forget, 拿不到回包). 但命令实际发出, firmware
    // 几 ms 后会切换. 所以这两个 mode 我们发完后 sleep + read_motor_state 验证.
    const bool need_verify_after_send = (mode == 0x00 || mode == 0x0F);

    for (int attempt = 1; attempt <= max_retry; ++attempt) {
        std::vector<int> failed;
        for (int mid : motor_ids) {
            auto s = ht.set_motor_mode(mid, mode);

            // async 下 brake/stop 返回 nullopt, 走二次验证
            if (need_verify_after_send && !s) {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                s = ht.read_motor_state(mid, 0.15);
            }

            const int got = s ? s->mode : -1;
            if (s && got == static_cast<int>(mode)) {
                std::cout << "    电机 " << mid << ": OK (mode=0x"
                          << std::hex << got << std::dec << ")";
                if (attempt > 1) std::cout << "  [重试 #" << attempt << "]";
                std::cout << "\n";
            } else {
                failed.push_back(mid);
                std::cout << "    电机 " << mid << ": 失败 (mode=0x"
                          << std::hex << got << " fault=0x"
                          << (s ? s->fault : 0) << std::dec << ")\n";
            }
        }
        if (failed.empty()) return true;
        if (attempt < max_retry) {
            std::cout << "    → 重试 " << failed.size() << " 个电机...\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
//  交互式主循环
// ---------------------------------------------------------------------------

static void interactive_loop(HightorqueSerial& ht, const RobotConfig& cfg) {
    const auto& motor_ids = cfg.motor_ids;

    // brake/free 状态. 进入这两个模式后, 下次 movement 命令会自动先 release
    // (= 切回 mode=10), 不需要用户手动 release.
    bool is_disabled = false;

    // movement 命令公共前置: 如果当前是 brake/free, 自动 release 到 mode=10.
    // 失败 (例如电机进 fault) 返回 false, 主循环跳过此次运动.
    auto ensure_position_mode = [&]() -> bool {
        if (!is_disabled) return true;
        std::cout << "  [自动] 当前处于 brake/free 状态, 切回位置控制...\n";
        if (switch_mode_all(ht, motor_ids, 0x0A, "位置控制", 2)) {
            is_disabled = false;
            // 给 cache 一点时间被新 read 填充
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            return true;
        }
        std::cout << "  [失败] 无法切回位置控制, 取消运动\n";
        return false;
    };

    while (true) {
        read_all(ht, motor_ids);

        std::cout << "\n  命令 (位置单位: 度):\n"
                  << "    <ID> <位置度>      单关节运动 (例: 1 36.0)\n"
                  << "    all <p1>..<pN>     " << motor_ids.size() << " 关节同时设置 (单位: 度)\n"
                  << "    home               所有关节回零\n"
                  << "    brake / b          全部刹车 (mode=0x0F, 三相短路阻尼)\n"
                  << "    free  / stop       全部断电 (mode=0x00, 可手动转动调位姿)\n"
                  << "    release / r        切回位置控制 (mode=0x0A, hold 当前位置)\n"
                  << "    stats              查看 TX/RX 统计\n"
                  << "    q                  退出\n"
                  << "    ESC                紧急停止 (运动中 / 输入中均有效)\n";
        if (is_disabled) {
            std::cout << "  ⚠️ 当前: brake/free 状态, 下次运动会自动 release\n";
        }

        const std::string cmd = strip_lower(input_line("\n  > "));

        if (cmd.empty()) continue;       // 空回车 / 粘贴 \r\n 残留 → 静默跳过
        if (cmd == "q") break;

        if (cmd == "stats") {
            std::cout << "  " << ht.get_stats().to_string() << "\n";
            continue;
        }

        // ---- brake / free / release ----
        if (cmd == "brake" || cmd == "b") {
            if (switch_mode_all(ht, motor_ids, 0x0F, "刹车 (短路阻尼)", /*retry*/ 1)) {
                is_disabled = true;
                std::cout << "  → 全部电机已刹车. 下次运动命令会自动 release.\n";
            }
            continue;
        }
        if (cmd == "free" || cmd == "stop") {
            if (switch_mode_all(ht, motor_ids, 0x00, "停止 (无电流, 可手动转)", /*retry*/ 1)) {
                is_disabled = true;
                std::cout << "  → 全部电机已断电. 现在可以手动摆位姿. 下次运动命令会自动 release.\n";
            }
            continue;
        }
        if (cmd == "release" || cmd == "r") {
            if (switch_mode_all(ht, motor_ids, 0x0A, "位置控制 hold", /*retry*/ 2)) {
                is_disabled = false;
                std::cout << "  → 全部电机已 release, 当前位置 hold 中.\n";
            }
            continue;
        }

        if (cmd == "home") {
            std::map<int, double> targets;
            for (int mid : motor_ids) {
                targets[mid] = 0.0;
                warn_if_out_of_limit(ht, mid, 0.0);
            }
            std::cout << "  目标: 全部回零\n";
            const std::string confirm = strip_lower(input_line("  确认? (Enter=执行, q=取消): "));
            if (confirm == "q") continue;
            if (!ensure_position_mode()) continue;
            move_all_async(ht, motor_ids, targets, cfg);
            continue;
        }

        if (starts_with(cmd, "all")) {
            const auto parts = split_ws(cmd);
            if (parts.size() != 1 + motor_ids.size()) {
                std::cout << "  格式错误, 需要 " << motor_ids.size() << " 个位置值 (单位: 度)\n";
                continue;
            }
            // 用户输入是"度", 内部 targets 统一存"圈" (协议原生)
            std::map<int, double> targets;
            bool ok = true;
            for (std::size_t i = 0; i < motor_ids.size(); ++i) {
                double pos_deg = 0.0;
                if (!parse_double(parts[1 + i], pos_deg)) { ok = false; break; }
                const int mid = motor_ids[i];
                warn_if_out_of_limit(ht, mid, pos_deg);
                targets[mid] = deg_to_turns(pos_deg);
            }
            if (!ok) {
                std::cout << "  位置值格式错误, 请输入数字 (单位: 度)\n";
                continue;
            }
            std::cout << "  目标: ";
            bool first = true;
            for (int mid : motor_ids) {
                if (!first) std::cout << ", ";
                std::cout << "M" << mid << "="
                          << std::showpos << std::fixed << std::setprecision(2)
                          << turns_to_deg(targets[mid]) << "°" << std::noshowpos;
                first = false;
            }
            std::cout << "\n";
            const std::string confirm = strip_lower(input_line("  确认? (Enter=执行, q=取消): "));
            if (confirm == "q") continue;
            if (!ensure_position_mode()) continue;
            move_all_async(ht, motor_ids, targets, cfg);
            continue;
        }

        // 单关节控制: "<id> <度>"
        const auto parts = split_ws(cmd);
        if (parts.size() == 2) {
            int    mid     = 0;
            double pos_deg = 0.0;
            if (!parse_int(parts[0], mid) || !parse_double(parts[1], pos_deg)) {
                std::cout << "  格式错误, 请输入: <电机ID> <位置(度)>\n";
                continue;
            }
            if (std::find(motor_ids.begin(), motor_ids.end(), mid) == motor_ids.end()) {
                std::cout << "  电机 " << mid << " 不在配置的电机列表中\n";
                continue;
            }
            warn_if_out_of_limit(ht, mid, pos_deg);
            const double pos_turns = deg_to_turns(pos_deg);
            std::cout << "  目标: 电机 " << mid << " -> "
                      << std::showpos << std::fixed << std::setprecision(2) << pos_deg
                      << std::noshowpos << " 度\n";
            const std::string confirm = strip_lower(input_line("  确认? (Enter=执行, q=取消): "));
            if (confirm == "q") continue;
            if (!ensure_position_mode()) continue;
            move_all_async(ht, motor_ids, {{mid, pos_turns}}, cfg);
            continue;
        }

        std::cout << "  未识别命令\n";
    }
}

// ---------------------------------------------------------------------------
//  配置加载 (失败回退到内置默认)
// ---------------------------------------------------------------------------

static RobotConfig load_or_default(const std::string& path) {
    try {
        auto cfg = RobotConfig::load(path);
        std::cout << "[配置] 从 '" << path << "' 加载\n";
        return cfg;
    } catch (const std::exception& e) {
        std::cout << "[配置] 无法加载 '" << path << "' (" << e.what() << ")\n"
                  << "       回退到内置默认值\n";
        RobotConfig cfg;
        cfg.port      = "COM14";
        cfg.baudrate  = 4'000'000u;
        cfg.motor_ids = {1, 2, 3, 4, 5, 6, 7};
        cfg.pos_unit  = PosUnit::Degrees;
        // 内部 limits 始终按"圈"存; 这里把度数限位除以 360 写成圈, 方便 cfg.apply_limits_to.
        // 兜底默认: 1=±144/108°, 2=±18/173°, 3=0/169°, 4-7=±90° (与原 Turns 兜底机械意义一致).
        cfg.limits = {
            {1, {deg_to_turns(-144.0), deg_to_turns(108.0)}},
            {2, {deg_to_turns( -18.0), deg_to_turns(173.0)}},
            {3, {deg_to_turns(   0.0), deg_to_turns(169.0)}},
            {4, {deg_to_turns( -90.0), deg_to_turns( 90.0)}},
            {5, {deg_to_turns( -90.0), deg_to_turns( 90.0)}},
            {6, {deg_to_turns( -90.0), deg_to_turns( 90.0)}},
            {7, {deg_to_turns( -90.0), deg_to_turns( 90.0)}},
        };
        return cfg;
    }
}

// ---------------------------------------------------------------------------
//  USB 串口自动识别
//  → 实现已抽到 include/port_picker.hpp, 由 hightorque::pick_serial_port 提供
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
//  main
// ---------------------------------------------------------------------------

int main(int argc, char** argv) {
#ifdef _WIN32
    // 把控制台输出切到 UTF-8, 让中文正常显示
    SetConsoleOutputCP(CP_UTF8);
#endif

    const std::string cfg_path = (argc > 1) ? argv[1] : "robot.cfg";
    RobotConfig cfg = load_or_default(cfg_path);

    if (cfg.motor_ids.empty()) {
        std::cerr << "[错误] 配置中 motor_ids 为空, 无可控关节, 退出.\n";
        return 1;
    }

    // ---- 自动选端口 (优先 cfg.port, 失败 fallback 到 USB 枚举) ----
    cfg.port = hightorque::pick_serial_port(cfg.port);

    // ---- 打开串口 ----
    std::unique_ptr<HightorqueSerial> ht_ptr;
    try {
        ht_ptr = std::make_unique<HightorqueSerial>(cfg.port, cfg.baudrate);
    } catch (const std::exception& e) {
        std::cerr << "\n[错误] 无法打开串口 " << cfg.port << ": " << e.what() << "\n"
                  << "       请检查: (1) 调试板 USB 是否插好\n"
                  << "                (2) 端口是否被其它程序占用 (例: PuTTY / Python 调试脚本)\n"
                  << "                (3) 在 " << cfg_path << " 中把 port= 改成 'auto' 或具体端口名\n";
        std::cout << "\n按 Enter 退出..." << std::flush;
        std::cin.get();
        return 1;
    }
    HightorqueSerial& ht = *ht_ptr;

    // ---- 一行把限位灌进驱动 (此后所有 set_pos_* 自动 clamp) ----
    cfg.apply_limits_to(ht);

    // ---- 启动横幅 ----
    std::cout << std::string(60, '=') << "\n"
              << "  多关节控制 (" << static_cast<int>(cfg.control_rate_hz)
              << "Hz 异步 + 一拖多 + S 曲线)\n"
              << "  端口: " << cfg.port << " @ " << cfg.baudrate << " bps\n"
              << "  电机: [";
    for (std::size_t i = 0; i < cfg.motor_ids.size(); ++i) {
        if (i) std::cout << ", ";
        std::cout << cfg.motor_ids[i];
    }
    std::cout << "]\n"
              << "  控制频率: " << cfg.control_rate_hz << " Hz (周期 "
              << std::fixed << std::setprecision(1) << (1000.0 / std::max(1.0, cfg.control_rate_hz))
              << " ms), 单段时长: " << cfg.trajectory_dt_s << " s, max_tqe_raw: "
              << cfg.max_torque_raw << "\n"
              << "  轨迹: S 曲线 (cosine ease-in-out), v_peak = π/2 · v_avg ≈ 1.57 · v_avg\n"
              << "  异步 RX: " << (cfg.use_async_rx ? "开 (推荐)" : "关 (兼容旧路径)") << "\n"
              << "  限位 (来自 " << cfg_path << "):\n";
    for (int mid : cfg.motor_ids) {
        double lo_t = 0.0, hi_t = 0.0;
        if (ht.get_position_limit_turns(mid, lo_t, hi_t)) {
            std::cout << "    电机 " << mid << ": ["
                      << std::showpos << std::fixed << std::setprecision(2)
                      << turns_to_deg(lo_t) << ", " << turns_to_deg(hi_t)
                      << std::noshowpos << "] 度\n";
        } else {
            std::cout << "    电机 " << mid << ": (未设软限位)\n";
        }
    }
    std::cout << std::string(60, '=') << "\n";

    // ---- 通信预检 (同步路径, 在启用 async 之前) ----
    std::cout << "\n--- 通信预检 ---\n";
    bool all_ok = true;
    for (int mid : cfg.motor_ids) {
        if (auto s = ht.read_motor_state(mid)) {
            std::cout << "  电机 " << mid << ": OK (pos="
                      << std::showpos << std::fixed << std::setprecision(2)
                      << turns_to_deg(s->position) << "°" << std::noshowpos << ")\n";
        } else {
            std::cout << "  电机 " << mid << ": 无响应!\n";
            all_ok = false;
        }
    }

    if (!all_ok) {
        std::cout << "\n有电机无响应, 请检查连接.\n";
        ht.close();
        return 1;
    }

    // ---- 切到位置/速度/力矩模式 (mode=10) ----
    //
    // 一拖多帧 (CAN ID 0x8090) 不含 mode 设置子帧, 上电默认 mode=0 (停止) 时电机
    // 会接收命令但不动. 必须先把每个电机切到 mode=10. 这一步必须在 enable_async_rx
    // 之前 — async 模式下 set_motor_mode 返回 nullopt 没法确认结果.
    std::cout << "\n--- 切到位置模式 (mode=10) ---\n";
    if (!switch_mode_all(ht, cfg.motor_ids, 0x0A, "位置控制", /*retry*/ 3)) {
        std::cout << "\n[失败] 有电机无法切到位置模式, 可能原因:\n"
                  << "  1) 电机进入 fault 状态 (fault 字段非 0): 上一次会话的 tqe=0 错配\n"
                  << "     可能让电机触发了保护. 解决: 把电机断电 5 秒后重新上电.\n"
                  << "  2) 电机机械卡死 / 撞到限位.\n"
                  << "  3) 通信不稳: 看上面 fault=0x00 但 mode=0 的电机, 可能是 USB 抖动.\n";
        ht.close();
        return 1;
    }

    // ---- 启用异步 RX (高频控制必备) + 预热 cache ----
    //
    // 注意: async RX 线程本身只接收, 不主动发查询. 所以 cache 不会自动持续刷新,
    // 必须由上层每次需要时调 read_motor_state() 或在控制环里发一拖多 (内含查询子帧).
    // 这里 warmup 用 read_motor_state 而不是 set_many — 因为 set_many 会带着 pos/vel/tqe
    // 实际控制电机, 而启动阶段我们只想 read 不想 move.
    if (cfg.use_async_rx) {
        std::cout << "\n[异步 RX] 启动后台接收线程...\n";
        ht.enable_async_rx();
        // 给 RX 线程 100ms 启动时间
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // 用单电机查询(只读, 不会动) 灌一遍 cache
        for (int mid : cfg.motor_ids) {
            (void)ht.read_motor_state(mid, 0.1);
        }
        const auto stats = ht.get_stats();
        std::cout << "[异步 RX] 预热完成. " << stats.to_string() << "\n";

        // 再快速验证 cache 真的有数据
        int cached = 0;
        for (int mid : cfg.motor_ids) if (ht.get_state(mid)) ++cached;
        if (cached < static_cast<int>(cfg.motor_ids.size())) {
            std::cout << "[异步 RX] 警告: cache 中只有 " << cached << "/" << cfg.motor_ids.size()
                      << " 个电机有数据. 通信可能不稳, 但程序仍可运行.\n";
        } else {
            std::cout << "[异步 RX] cache 满载 " << cached << "/" << cfg.motor_ids.size() << " ✓\n";
        }
    }

    int exit_code = 0;
    try {
        interactive_loop(ht, cfg);
    } catch (const EmergencyStop&) {
        std::cout << "\n\n紧急停止 (ESC / Ctrl+Q)!\n";
        exit_code = 1;
    } catch (const std::exception& e) {
        std::cout << "\n\n异常: " << e.what() << "\n";
        exit_code = 2;
    }

    // 收尾
    if (ht.is_async_rx()) ht.disable_async_rx();
    for (int mid : cfg.motor_ids) {
        try { ht.stop(mid); } catch (...) {}
    }
    ht.close();

    std::cout << "\n程序结束 (exit=" << exit_code << ")\n";
    return exit_code;
}
