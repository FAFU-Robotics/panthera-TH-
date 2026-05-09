// =============================================================================
//  14_set_limits.cpp
//  软限位标定/编辑工具 — 交互式 C++ 程序
//
//  典型工作流:
//    1. 程序启动: 读 robot.cfg, 把已有 limits 灌进 driver, 显示总览
//    2. 把电机切到 free (mode=0), 用手把目标关节摆到机械下限位置
//    3. 在主菜单里输入电机 ID, 进入子菜单
//    4. 输入 'l' (lo) → 当前位置写入 lo
//    5. 把电机摆到机械上限位置, 输入 'h' (hi) → 当前位置写入 hi
//    6. 重复 3~5 标定其它关节
//    7. 主菜单输入 's' → 把当前所有限位写回 robot.cfg (原文件备份为 .bak)
//    8. 主菜单输入 'q' 退出
//
//  支持的所有操作:
//    主菜单:
//      l                列出所有电机当前限位 + 实时位置
//      <数字>           选定电机, 进入子菜单
//      c                清空全部限位
//      s                保存到 robot.cfg
//      q                退出 (不会保存!)
//
//    子菜单 (选定 M<id>):
//      l                把当前位置设为 lo (下限)
//      h                把当前位置设为 hi (上限)
//      m <lo> <hi>      手动输入新 lo/hi (按 cfg.pos_unit 单位)
//      m lo <lo>        只改 lo
//      m hi <hi>        只改 hi
//      d                禁用本关节限位
//      b                返回主菜单
//
//  单位:
//    所有用户输入/输出都按 cfg.pos_unit (turns / radians / degrees);
//    driver 内部存储统一是"圈". cfg 文件保存时也按原 pos_unit 写回.
//
//  机械臂安全提醒:
//    - 标定前最好把电机切到 free 模式 (本程序启动时会自动 stop 一次), 让你能用手摆
//    - 重力关节 (肩 / 肘 / 一般是 2 号 / 3 号) free 后会下坠, 请双手扶稳
//    - 限位是"软"的: driver 在 set_pos_* 时 clamp, 不会主动顶住关节
//    - 标定完务必输 's' 保存, 不然下次启动还是 cfg 里的旧值
// =============================================================================

#include "hightorque_serial.hpp"
#include "robot_config.hpp"
#include "port_picker.hpp"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#ifdef _WIN32
  #ifndef NOMINMAX
  #define NOMINMAX
  #endif
  #include <windows.h>
#endif

using hightorque::HightorqueSerial;
using hightorque::PosUnit;
using hightorque::RobotConfig;

// ----------------------------------------------------------------------------
//  小工具
// ----------------------------------------------------------------------------

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

static bool parse_int(const std::string& s, int& out) {
    try {
        std::size_t idx = 0;
        out = std::stoi(s, &idx);
        return idx == s.size();
    } catch (...) { return false; }
}

static bool parse_double(const std::string& s, double& out) {
    try {
        std::size_t idx = 0;
        out = std::stod(s, &idx);
        return idx == s.size();
    } catch (...) { return false; }
}

static const char* unit_str(PosUnit u) {
    return (u == PosUnit::Turns)   ? "turns"
         : (u == PosUnit::Radians) ? "rad"
                                   : "deg";
}

static void sleep_seconds(double s) {
    if (s <= 0) return;
    std::this_thread::sleep_for(
        std::chrono::microseconds(static_cast<long long>(s * 1'000'000)));
}

// ----------------------------------------------------------------------------
//  全局: 总览 (列限位表)
// ----------------------------------------------------------------------------

static double read_pos_turns_or_nan(HightorqueSerial& ht, int mid) {
    auto s = ht.read_motor_state(mid, 0.3);
    if (!s) return std::numeric_limits<double>::quiet_NaN();
    return s->position;
}

static void print_overview(HightorqueSerial& ht, const RobotConfig& cfg) {
    const PosUnit u = cfg.pos_unit;
    std::cout << "\n--- 当前限位 (单位: " << unit_str(u) << ") ---\n";
    std::cout << "    " << std::left << std::setw(8) << "ID"
              << std::setw(14) << "现位置"
              << std::setw(14) << "lo"
              << std::setw(14) << "hi"
              << std::setw(8)  << "状态" << "\n";

    for (int mid : cfg.motor_ids) {
        double lo_t, hi_t;
        const bool has = ht.get_position_limit_turns(mid, lo_t, hi_t);
        const double pos_t = read_pos_turns_or_nan(ht, mid);

        std::cout << "    M" << std::left << std::setw(7) << mid;

        // 现位置
        if (std::isnan(pos_t)) {
            std::cout << std::left << std::setw(14) << "—";
        } else {
            std::ostringstream ps;
            ps << std::showpos << std::fixed << std::setprecision(3)
               << hightorque::from_turns(pos_t, u);
            std::cout << std::left << std::setw(14) << ps.str();
        }

        if (has) {
            std::ostringstream lo, hi;
            lo << std::showpos << std::fixed << std::setprecision(3)
               << hightorque::from_turns(lo_t, u);
            hi << std::showpos << std::fixed << std::setprecision(3)
               << hightorque::from_turns(hi_t, u);
            std::cout << std::left << std::setw(14) << lo.str()
                      << std::left << std::setw(14) << hi.str()
                      << std::left << std::setw(8)  << "ENABLED";
        } else {
            std::cout << std::left << std::setw(14) << "—"
                      << std::left << std::setw(14) << "—"
                      << std::left << std::setw(8)  << "(none)";
        }
        std::cout << "\n";
    }
    std::cout << "\n";
}

// ----------------------------------------------------------------------------
//  把当前 driver 限位写回 cfg 文件
//
//  策略 (跟 panthera_web/app.py 里的实现一致):
//    1. 读原文件每行
//    2. 凡是 "limits.<id> = ..." 的行, 整体替换成 driver 当前的 limits
//       (第一条 limits.* 出现的位置一次性写入新表, 后续旧 limits.* 行删掉)
//    3. 其它行 (注释 / port / baudrate / pos_unit / control_rate_hz / ...) 全部保留
//    4. 原文件备份成 .bak
//
//  写入时按 cfg.pos_unit 选单位; cfg 没有的电机 (motor_ids 之外) 不写.
// ----------------------------------------------------------------------------

static bool save_limits_to_cfg(const std::string& path,
                               HightorqueSerial& ht,
                               const RobotConfig& cfg) {
    // 1) 收集 driver 当前所有限位 (按 motor_id 升序)
    std::map<int, std::pair<double, double>> lims;   // motor_id -> (lo, hi) 圈
    for (int mid : cfg.motor_ids) {
        double lo_t, hi_t;
        if (ht.get_position_limit_turns(mid, lo_t, hi_t)) {
            lims[mid] = {lo_t, hi_t};
        }
    }

    // 2) 按 cfg.pos_unit 格式化每条 limits.<id> 行
    auto fmt_line = [&](int mid, double lo_t, double hi_t) -> std::string {
        std::ostringstream ss;
        const double lo_u = hightorque::from_turns(lo_t, cfg.pos_unit);
        const double hi_u = hightorque::from_turns(hi_t, cfg.pos_unit);
        ss << "limits." << std::left << std::setw(3) << mid
           << " = "
           << std::fixed << std::setprecision(4) << std::setw(9) << lo_u
           << ", "
           << std::fixed << std::setprecision(4) << std::setw(9) << hi_u;
        return ss.str();
    };

    std::vector<std::string> new_limit_lines;
    new_limit_lines.reserve(lims.size());
    for (const auto& [mid, lim] : lims) {
        new_limit_lines.push_back(fmt_line(mid, lim.first, lim.second));
    }

    // 3) 备份原文件 (如果存在)
    std::ifstream fin(path);
    if (!fin) {
        // 文件不存在: 直接写一个最小可用的 cfg
        std::ofstream fout(path);
        if (!fout) {
            std::cout << "  ! 无法打开输出文件: " << path << "\n";
            return false;
        }
        fout << "# robot.cfg (由 14_set_limits 自动生成)\n"
             << "port      = " << cfg.port     << "\n"
             << "baudrate  = " << cfg.baudrate << "\n"
             << "motor_ids = ";
        for (std::size_t i = 0; i < cfg.motor_ids.size(); ++i) {
            if (i) fout << ", ";
            fout << cfg.motor_ids[i];
        }
        fout << "\npos_unit  = " << unit_str(cfg.pos_unit) << "\n\n";
        for (const auto& l : new_limit_lines) fout << l << "\n";
        std::cout << "  -> 新建配置文件: " << path
                  << " (写入 " << new_limit_lines.size() << " 条限位)\n";
        return true;
    }

    std::vector<std::string> old_lines;
    {
        std::string line;
        while (std::getline(fin, line)) old_lines.push_back(std::move(line));
    }
    fin.close();

    // 备份
    const std::string bak = path + ".bak";
    {
        std::ofstream fbak(bak);
        if (fbak) {
            for (const auto& l : old_lines) fbak << l << "\n";
        }
    }

    // 4) 行级处理: 找 limits.* 行, 替换
    auto is_limit_line = [](const std::string& raw) -> bool {
        std::string s;
        // 去前导空白
        std::size_t i = 0;
        while (i < raw.size() && std::isspace(static_cast<unsigned char>(raw[i]))) ++i;
        // 注释行
        if (i >= raw.size() || raw[i] == '#' || raw[i] == ';') return false;
        // 找 '='
        const auto eq = raw.find('=', i);
        if (eq == std::string::npos) return false;
        std::string head = raw.substr(i, eq - i);
        // 去尾部空白
        while (!head.empty() && std::isspace(static_cast<unsigned char>(head.back()))) head.pop_back();
        // 转小写
        std::transform(head.begin(), head.end(), head.begin(),
                       [](unsigned char c) { return std::tolower(c); });
        return head.rfind("limits.", 0) == 0;
    };

    std::vector<std::string> out;
    out.reserve(old_lines.size() + 4);
    bool seen_limit = false;
    for (const auto& line : old_lines) {
        if (is_limit_line(line)) {
            if (!seen_limit) {
                for (const auto& nl : new_limit_lines) out.push_back(nl);
                seen_limit = true;
            }
            // 旧的 limits.* 全部丢弃
        } else {
            out.push_back(line);
        }
    }
    if (!seen_limit && !new_limit_lines.empty()) {
        // 原文件没有 limits.*, 在文件末尾追加
        if (!out.empty() && !out.back().empty()) out.push_back("");
        out.push_back("# ---- 软限位 (由 14_set_limits 写入, 单位 " +
                      std::string(unit_str(cfg.pos_unit)) + ") ----");
        for (const auto& nl : new_limit_lines) out.push_back(nl);
    }

    // 5) 写回
    std::ofstream fout(path);
    if (!fout) {
        std::cout << "  ! 无法打开输出文件: " << path << "\n";
        return false;
    }
    for (const auto& l : out) fout << l << "\n";

    std::cout << "  -> 已写入 " << path
              << " (" << new_limit_lines.size() << " 条限位, 单位 "
              << unit_str(cfg.pos_unit) << ")\n"
              << "  -> 原文件备份: " << bak << "\n";
    return true;
}

// ----------------------------------------------------------------------------
//  应用一组新限位 (校验 + 调 enable_position_limit)
//  new_lo_t / new_hi_t 是"圈". 出错时返回 false 并打印.
// ----------------------------------------------------------------------------

static bool apply_limit(HightorqueSerial& ht, int mid,
                        double new_lo_t, double new_hi_t,
                        const RobotConfig& cfg) {
    if (new_lo_t > new_hi_t) {
        std::cout << "  ! 失败: lo=" << hightorque::from_turns(new_lo_t, cfg.pos_unit)
                  << " > hi=" << hightorque::from_turns(new_hi_t, cfg.pos_unit)
                  << " (" << unit_str(cfg.pos_unit) << ")\n";
        return false;
    }
    try {
        ht.enable_position_limit(mid, new_lo_t, new_hi_t, PosUnit::Turns);
    } catch (const std::exception& e) {
        std::cout << "  ! enable_position_limit 异常: " << e.what() << "\n";
        return false;
    }
    std::cout << "  -> M" << mid << " 限位更新为 ["
              << std::fixed << std::setprecision(4)
              << hightorque::from_turns(new_lo_t, cfg.pos_unit) << ", "
              << hightorque::from_turns(new_hi_t, cfg.pos_unit) << "] "
              << unit_str(cfg.pos_unit) << "\n";
    return true;
}

// ----------------------------------------------------------------------------
//  PositionWatcher
//    后台线程, 每 ~200ms 读一次电机位置, 把"现位置:" 那一行原地刷新.
//    实现: ANSI 转义 \033[s (保存光标) → \033[<n>A (上移 n 行) → \r\033[K
//          (清整行) → 写新位置 → \033[u (恢复光标), 这样用户在 prompt 处
//          已敲的字符不会被打断.
//    用法:
//        PositionWatcher w(ht, mid, cfg, /*lines_above_prompt=*/1);
//        std::getline(std::cin, line);     // 后台线程同时刷新
//        w.stop();                         // 拿到输入后停止
//    线程安全:
//        HightorqueSerial::read_motor_state 内部用 tx_mtx_ / cache_mtx_
//        保护, 多线程并发读 OK; PositionWatcher 自己只用 std::atomic.
// ----------------------------------------------------------------------------

class PositionWatcher {
public:
    PositionWatcher(HightorqueSerial& ht, int mid, const RobotConfig& cfg,
                    int lines_above_prompt)
        : ht_(ht), mid_(mid), cfg_(cfg), up_(lines_above_prompt) {
        // 后台线程
        thr_ = std::thread([this]() { run(); });
    }

    ~PositionWatcher() { stop(); }

    void stop() {
        if (stop_.exchange(true)) return;
        if (thr_.joinable()) thr_.join();
    }

private:
    void run() {
        // 第一帧立刻刷新一次 (启动后用户看到的不是占位 "..."),
        // 然后每 ~200ms 刷新.
        while (!stop_.load()) {
            // read_motor_state 自己 timeout 0.25s, 比 sleep 间隔短即可
            const double pos_t = read_pos_turns_or_nan(ht_, mid_);

            std::ostringstream oss;
            // \033[s 存光标, \033[<n>A 上移 n 行, \r 回行首, \033[K 清到行尾
            oss << "\033[s\033[" << up_ << "A\r\033[K"
                << "  现位置: ";
            if (std::isnan(pos_t)) {
                oss << "无响应";
            } else {
                oss << std::showpos << std::fixed << std::setprecision(4)
                    << hightorque::from_turns(pos_t, cfg_.pos_unit)
                    << std::noshowpos << " " << unit_str(cfg_.pos_unit);
            }
            oss << "\033[u";   // 恢复光标 (回到 prompt 处)
            std::cout << oss.str() << std::flush;

            // 退出前能提前响应 stop_
            for (int i = 0; i < 4 && !stop_.load(); ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        }
    }

    HightorqueSerial& ht_;
    int               mid_;
    const RobotConfig& cfg_;
    int               up_;             // prompt 离"现位置"那一行的行数
    std::atomic<bool> stop_{false};
    std::thread       thr_;
};

// ----------------------------------------------------------------------------
//  子菜单: 选定 M<mid> 后的操作
// ----------------------------------------------------------------------------

static void motor_submenu(HightorqueSerial& ht, int mid, const RobotConfig& cfg) {
    while (true) {
        // 读限位 (静态 — 用户没动它的时候不会变)
        double lo_t = 0, hi_t = 0;
        const bool has = ht.get_position_limit_turns(mid, lo_t, hi_t);

        std::cout << "\n=== M" << mid << " ===\n";
        if (has) {
            std::cout << "  限位: [" << std::fixed << std::setprecision(4)
                      << hightorque::from_turns(lo_t, cfg.pos_unit) << ", "
                      << hightorque::from_turns(hi_t, cfg.pos_unit) << "] "
                      << unit_str(cfg.pos_unit) << "\n";
        } else {
            std::cout << "  限位: (未设, driver 不会 clamp)\n";
        }
        // 占位行 — 后台线程会用 \033[<n>A 上移到这一行, 用 \033[K 清空再写
        std::cout << "  现位置: 读取中...\n";
        std::cout << "  操作:\n"
                  << "    l                把当前位置设为 lo (下限) [实时刷新中]\n"
                  << "    h                把当前位置设为 hi (上限) [实时刷新中]\n"
                  << "    m <lo> <hi>      手动输入新限位 (单位 "
                                          << unit_str(cfg.pos_unit) << ")\n"
                  << "    m lo <值>        只改下限\n"
                  << "    m hi <值>        只改上限\n"
                  << "    d                禁用本关节限位\n"
                  << "    b                返回主菜单\n"
                  << "  > " << std::flush;

        // 启动后台监控: prompt 离"现位置"那一行差 9 行
        // (操作行 7 行 + "操作:" 1 行 + prompt 自己 1 行 = 9, 但 prompt 是
        //  当前行 — \r 回行首再上移 9 行刚好回到"现位置")
        PositionWatcher watcher(ht, mid, cfg, /*lines_above_prompt=*/9);

        std::string line;
        const bool got = static_cast<bool>(std::getline(std::cin, line));

        // 用户敲完 Enter, 立刻停止刷新 (避免输出跟下面的结果交错)
        watcher.stop();

        if (!got) return;
        const auto tokens = split_ws(strip_lower(line));
        if (tokens.empty()) continue;
        const std::string& cmd = tokens[0];

        if (cmd == "b" || cmd == "back" || cmd == "q") return;

        if (cmd == "l" || cmd == "h") {
            // l/h 把"当前位置"设为 lo/hi — 这里要再读一次, 不能用 watcher 内
            // 部最后一帧 (因为 watcher 没暴露). 这一次读会阻塞最多 0.3s, 不
            // 影响交互.
            const double pos_t = read_pos_turns_or_nan(ht, mid);
            if (std::isnan(pos_t)) {
                std::cout << "  ! 当前位置无法读取, 操作放弃\n";
                continue;
            }
            // 没设过限位时, 给另一端一个保守的极宽值 (±10 圈)
            double cur_lo = has ? lo_t : -10.0;
            double cur_hi = has ? hi_t :  10.0;
            double new_lo = (cmd == "l") ? pos_t  : cur_lo;
            double new_hi = (cmd == "h") ? pos_t  : cur_hi;
            apply_limit(ht, mid, new_lo, new_hi, cfg);
        }
        else if (cmd == "m" && tokens.size() == 3 && (tokens[1] == "lo" || tokens[1] == "hi")) {
            double v;
            if (!parse_double(tokens[2], v)) {
                std::cout << "  ! 数字解析失败: " << tokens[2] << "\n";
                continue;
            }
            const double v_t = hightorque::to_turns(v, cfg.pos_unit);
            double cur_lo = has ? lo_t : -10.0;
            double cur_hi = has ? hi_t :  10.0;
            double new_lo = (tokens[1] == "lo") ? v_t : cur_lo;
            double new_hi = (tokens[1] == "hi") ? v_t : cur_hi;
            apply_limit(ht, mid, new_lo, new_hi, cfg);
        }
        else if (cmd == "m" && tokens.size() == 3) {
            double lo_v, hi_v;
            if (!parse_double(tokens[1], lo_v) || !parse_double(tokens[2], hi_v)) {
                std::cout << "  ! 数字解析失败 (期望: m <lo> <hi>)\n";
                continue;
            }
            const double new_lo = hightorque::to_turns(lo_v, cfg.pos_unit);
            const double new_hi = hightorque::to_turns(hi_v, cfg.pos_unit);
            apply_limit(ht, mid, new_lo, new_hi, cfg);
        }
        else if (cmd == "d" || cmd == "disable") {
            std::cout << "  确认禁用 M" << mid << " 的软限位? (yes/no): " << std::flush;
            std::string ans;
            std::getline(std::cin, ans);
            if (strip_lower(ans) != "yes") {
                std::cout << "  已取消\n";
                continue;
            }
            ht.disable_position_limit(mid);
            std::cout << "  -> M" << mid << " 限位已禁用\n";
        }
        else {
            std::cout << "  ! 未识别命令\n";
        }
    }
}

// ----------------------------------------------------------------------------
//  主菜单
// ----------------------------------------------------------------------------

static void main_menu(HightorqueSerial& ht, const RobotConfig& cfg,
                      const std::string& cfg_path) {
    while (true) {
        std::cout << "\n========================================\n"
                  << "  主菜单 (单位: " << unit_str(cfg.pos_unit) << ")\n"
                  << "    l        列出所有限位 + 实时位置\n"
                  << "    <数字>   选定电机 (例: 3)\n"
                  << "    c        清空全部限位\n"
                  << "    s        保存到 " << cfg_path << "\n"
                  << "    q        退出 (不保存!)\n"
                  << "  > " << std::flush;

        std::string line;
        if (!std::getline(std::cin, line)) return;
        const std::string s = strip_lower(line);
        if (s.empty()) continue;

        if (s == "q" || s == "quit" || s == "exit") return;

        if (s == "l" || s == "list") {
            print_overview(ht, cfg);
            continue;
        }

        if (s == "c" || s == "clear") {
            std::cout << "  确认清空全部限位? 之后所有 set_pos_* 都不再 clamp! (yes/no): "
                      << std::flush;
            std::string ans;
            std::getline(std::cin, ans);
            if (strip_lower(ans) != "yes") {
                std::cout << "  已取消\n";
                continue;
            }
            ht.clear_all_position_limits();
            std::cout << "  -> 全部限位已清空 (driver 端). 记得 's' 保存到 cfg!\n";
            continue;
        }

        if (s == "s" || s == "save") {
            std::cout << "  即将把当前 driver 限位写回 " << cfg_path
                      << " (原文件备份为 .bak), 确认? (yes/no): " << std::flush;
            std::string ans;
            std::getline(std::cin, ans);
            if (strip_lower(ans) != "yes") {
                std::cout << "  已取消\n";
                continue;
            }
            save_limits_to_cfg(cfg_path, ht, cfg);
            continue;
        }

        // 数字 → 选电机
        int mid;
        if (parse_int(s, mid)) {
            if (std::find(cfg.motor_ids.begin(), cfg.motor_ids.end(), mid) ==
                cfg.motor_ids.end()) {
                std::cout << "  ! 电机 " << mid
                          << " 不在配置 motor_ids 列表中, 已忽略\n";
                continue;
            }
            motor_submenu(ht, mid, cfg);
            continue;
        }

        std::cout << "  ! 未识别命令\n";
    }
}

// ----------------------------------------------------------------------------
//  main
// ----------------------------------------------------------------------------

int main(int argc, char** argv) {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
    // 启用 ANSI 转义 (\033[s, \033[u, \033[A, \033[K) — 实时位置刷新需要它.
    // Windows 10+ cmd / PowerShell / Windows Terminal 都支持; 失败也不致命.
    {
        HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
        DWORD mode = 0;
        if (h != INVALID_HANDLE_VALUE && GetConsoleMode(h, &mode)) {
            SetConsoleMode(h, mode | 0x0004 /* ENABLE_VIRTUAL_TERMINAL_PROCESSING */);
        }
    }
#endif

    // 命令行: 14_set_limits.exe [robot.cfg]
    const std::string cfg_path = (argc > 1) ? argv[1] : "robot.cfg";

    // 1) 加载 cfg
    RobotConfig cfg;
    try {
        cfg = RobotConfig::load(cfg_path);
        std::cout << "[配置] 从 '" << cfg_path << "' 加载\n";
    } catch (const std::exception& e) {
        std::cerr << "\n[错误] 加载配置失败: " << e.what() << "\n"
                  << "       请确认 " << cfg_path << " 存在 (从 11_multi_joint_control 同目录复制).\n";
        std::cout << "\n按 Enter 退出..." << std::flush;
        std::cin.get();
        return 1;
    }

    if (cfg.motor_ids.empty()) {
        std::cerr << "[错误] 配置中 motor_ids 为空, 无可控关节, 退出.\n";
        return 1;
    }

    // 2) 选串口
    cfg.port = hightorque::pick_serial_port(cfg.port);

    // 3) 打开
    std::unique_ptr<HightorqueSerial> ht_ptr;
    try {
        ht_ptr = std::make_unique<HightorqueSerial>(cfg.port, cfg.baudrate);
    } catch (const std::exception& e) {
        std::cerr << "\n[错误] 无法打开串口 " << cfg.port << ": " << e.what() << "\n"
                  << "       请检查: (1) 调试板 USB 是否插好  "
                  << "(2) 是否被其它程序占用 (e.g. 11_multi_joint_control / panthera_web)\n";
        std::cout << "\n按 Enter 退出..." << std::flush;
        std::cin.get();
        return 1;
    }
    HightorqueSerial& ht = *ht_ptr;

    // 4) 把 cfg 里的限位灌进 driver (作为初始值, 用户可以在此基础上改)
    cfg.apply_limits_to(ht);

    // 5) 把所有电机切到 free 模式, 用户可以手动转动关节定位
    //    (重力关节会下坠, 提醒一下)
    std::cout << "\n========================================\n"
              << "  软限位标定工具 14_set_limits\n"
              << "  端口:    " << cfg.port << " @ " << cfg.baudrate << " bps\n"
              << "  电机:    [";
    for (std::size_t i = 0; i < cfg.motor_ids.size(); ++i) {
        if (i) std::cout << ", ";
        std::cout << cfg.motor_ids[i];
    }
    std::cout << "]\n"
              << "  单位:    " << unit_str(cfg.pos_unit) << "\n"
              << "  cfg 文件: " << cfg_path << "\n"
              << "========================================\n"
              << "\n!! 即将把所有电机切到 free 模式 (mode=0), 你可以用手转关节定位.\n"
              << "   重力关节 (一般是 2/3 号肩肘) 会下坠, 请双手扶稳!\n"
              << "   按 Enter 继续, 或 Ctrl+C 退出..." << std::flush;
    std::cin.get();

    for (int mid : cfg.motor_ids) {
        try { ht.stop(mid); } catch (...) {}
    }
    sleep_seconds(0.3);

    // 6) 总览
    print_overview(ht, cfg);

    // 7) 主循环
    int exit_code = 0;
    try {
        main_menu(ht, cfg, cfg_path);
    } catch (const std::exception& e) {
        std::cerr << "\n[异常] " << e.what() << "\n";
        exit_code = 2;
    }

    // 8) 退出: 保持 free 模式 (用户可以继续手动操作机械臂)
    for (int mid : cfg.motor_ids) {
        try { ht.stop(mid); } catch (...) {}
    }
    ht.close();
    std::cout << "\n=== 程序结束 ===\n";
    return exit_code;
}
