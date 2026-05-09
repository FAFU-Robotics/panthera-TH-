// =============================================================================
//  09_set_zero.cpp
//  重置零点 — C++ 端口 (支持单关节 / 多关节 / 全部关节交互式选择)
//  基于 panthera_python/scripts/motor_example/09_set_zero.py 扩展
//
//  流程:
//    0. 交互选择要重置零点的关节 ID
//    1. 把所选电机切到 mode=0x00 (free), 关节可以用手随意摆动到目标零点姿态
//    2. 读取并打印当前电机状态 (5 次, 让你边摆边看实际位置)
//    3. 二次确认 (输入 yes 才执行)
//    4. 对每个选中关节依次执行 reset_zero -> save_config -> motor_reset
//    5. 等待重启
//    6. 再次切到 free + 读状态, 确认零位已更新且关节仍可手动检查
//
//  机械臂安全提醒 (多关节同时重置时尤其重要!):
//    - 重置零点会改变电机坐标系, 影响所有后续位置指令!
//    - 阶段 1 切到 free 时, 肩/肘 (2/3 号) 关节会因重力下坠 — 请双手扶稳
//    - 请用手把每个关节摆到目标零点姿态, 然后再输入 yes
//    - motor_reset 会导致电机短暂失力, 重力关节再次下坠 — 请继续扶稳
//    - 本脚本逐个串行执行 (不是并行), 每两个电机间隔 1.5s 给重力下坠预留反应时间
// =============================================================================

#include "hightorque_serial.hpp"
#include "port_picker.hpp"     // hightorque::pick_serial_port() — USB 自动识别

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <iostream>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
  #ifndef NOMINMAX
  #define NOMINMAX
  #endif
  #include <windows.h>
#endif

using hightorque::HightorqueSerial;

// ---------------------------------------------------------------------------
//  配置
//
//  PORT_PREFERRED 用 "auto" 时, 程序自动从系统 USB 串口里选一个;
//  也可以传命令行参数指定具体端口名 (例: 09_set_zero.exe COM14).
// ---------------------------------------------------------------------------

static const std::string PORT_PREFERRED = "auto";
static const uint32_t    BAUDRATE       = 4'000'000u;

// 可选的全部关节 (机械臂上接的所有电机 ID)
static const std::vector<int> ALL_MOTOR_IDS = {1, 2, 3, 4, 5, 6, 7};

// 关节间停顿 (秒): 单个关节 motor_reset 后, 等待这么久再处理下一个,
// 给重力下坠的关节预留反应时间.
static constexpr double PER_MOTOR_PAUSE = 1.5;

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

// ---------------------------------------------------------------------------
//  辅助: 打印一组状态
// ---------------------------------------------------------------------------

static void print_states(HightorqueSerial& ht,
                         const std::vector<int>& motor_ids,
                         const std::string& label = "") {
    if (!label.empty()) std::cout << "\n" << label << "\n";

    for (int mid : motor_ids) {
        const auto state = ht.read_motor_state(mid);
        if (!state) {
            std::cout << "  电机 " << mid << ": 无响应\n";
            continue;
        }
        // 协议原生单位是"圈" / "转/秒", 这里乘 360 显示为"度" / "度/秒"
        const double pos_deg = state->position * 360.0;
        const double vel_dps = state->velocity * 360.0;
        std::cout << "  电机 " << mid << "  "
                  << "位置: " << std::showpos << std::fixed << std::setprecision(2)
                              << std::setw(8) << pos_deg << std::noshowpos << " 度  "
                  << "速度: " << std::showpos << std::fixed << std::setprecision(2)
                              << std::setw(8) << vel_dps << std::noshowpos << " 度/秒  "
                  << "力矩: " << std::showpos << std::setw(6)
                              << static_cast<int>(state->torque) << std::noshowpos << "  "
                  << "模式: " << state->mode << "  "
                  << "故障: 0x" << std::hex << std::uppercase << std::setw(2)
                                << std::setfill('0') << state->fault
                                << std::dec << std::setfill(' ') << "\n";
    }
}

static void sleep_seconds(double s) {
    if (s <= 0) return;
    std::this_thread::sleep_for(
        std::chrono::microseconds(static_cast<long long>(s * 1'000'000)));
}

// ---------------------------------------------------------------------------
//  阶段 0: 交互选择关节
//
//  输入示例:
//    回车 / "all"     -> 全部 7 个关节
//    "3"              -> 只重置 3 号
//    "1 3 5" 或 "1,3,5" -> 重置 1/3/5 号
//    "q"              -> 退出
// ---------------------------------------------------------------------------

static std::vector<int> ask_motor_ids() {
    std::cout << "可重置的关节: ";
    for (std::size_t i = 0; i < ALL_MOTOR_IDS.size(); ++i) {
        if (i) std::cout << " ";
        std::cout << ALL_MOTOR_IDS[i];
    }
    std::cout << "\n"
              << "选择要重置零点的关节 (回车=全部, 例: '3' 或 '2 3 5', 'q'=退出):\n"
              << "  > " << std::flush;

    std::string line;
    if (!std::getline(std::cin, line)) return {};       // EOF

    const std::string s = strip_lower(line);

    if (s == "q") return {};                            // 用户主动退出
    if (s.empty() || s == "all") return ALL_MOTOR_IDS;

    // 把 ',' / ';' 当成空白, 然后按空白拆分
    std::string norm = s;
    for (char& c : norm) {
        if (c == ',' || c == ';') c = ' ';
    }
    const auto tokens = split_ws(norm);

    std::set<int> uniq;        // 去重 + 自动按 ID 升序
    std::vector<int> bad_input;
    std::vector<int> not_in_list;

    const std::set<int> all_set(ALL_MOTOR_IDS.begin(), ALL_MOTOR_IDS.end());
    for (const auto& tok : tokens) {
        try {
            std::size_t idx = 0;
            const int mid = std::stoi(tok, &idx);
            if (idx != tok.size()) { bad_input.push_back(0); continue; }
            if (all_set.find(mid) == all_set.end()) { not_in_list.push_back(mid); continue; }
            uniq.insert(mid);
        } catch (...) {
            bad_input.push_back(0);
        }
    }

    if (!bad_input.empty()) {
        std::cout << "  ! 输入有非数字 token, 已忽略\n";
    }
    if (!not_in_list.empty()) {
        std::cout << "  ! 以下 ID 不在可重置列表中, 已忽略:";
        for (int v : not_in_list) std::cout << " " << v;
        std::cout << "\n";
    }
    if (uniq.empty()) {
        std::cout << "  ! 没有有效的关节 ID, 退出\n";
        return {};
    }

    return std::vector<int>(uniq.begin(), uniq.end());
}

// ---------------------------------------------------------------------------
//  main
// ---------------------------------------------------------------------------

int main(int argc, char** argv) {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif

    // 命令行参数优先于内置默认值; 都没指定就 "auto"
    const std::string preferred = (argc > 1) ? std::string(argv[1]) : PORT_PREFERRED;
    const std::string port = hightorque::pick_serial_port(preferred);

    std::unique_ptr<HightorqueSerial> ht_ptr;
    try {
        ht_ptr = std::make_unique<HightorqueSerial>(port, BAUDRATE);
    } catch (const std::exception& e) {
        std::cerr << "\n[错误] 无法打开串口 " << port << ": " << e.what() << "\n"
                  << "       请检查: (1) 调试板 USB 是否插好  "
                  << "(2) 是否被其它程序占用\n"
                  << "       命令行可指定端口: 09_set_zero.exe COM14\n";
        std::cout << "\n按 Enter 退出..." << std::flush;
        std::cin.get();
        return 1;
    }
    HightorqueSerial& ht = *ht_ptr;

    std::cout << "=== 重置零点流程 ===\n\n";

    // ----- 阶段 0: 选择要重置的关节 -----
    std::cout << "--- 阶段 0: 选择关节 ---\n";
    const std::vector<int> motor_ids = ask_motor_ids();
    if (motor_ids.empty()) {
        std::cout << "\n未选择任何关节, 程序结束.\n";
        ht.close();
        return 0;
    }
    std::cout << "  -> 已选择 " << motor_ids.size() << " 个关节:";
    for (int mid : motor_ids) std::cout << " " << mid;
    std::cout << "\n";

    // ----- 阶段 1: 切到 free 模式, 让用户用手摆姿势 -----
    // 上一个程序可能把电机锁在 mode=10 (位置) 或 0x0F (brake), 这里强制切回 0x00.
    // 安全提醒: 重力关节 (2/3 号) 在解锁瞬间会下坠 — 请扶稳!
    std::cout << "\n--- 阶段 1: 切到 free 模式, 可手动转动 ---\n";
    std::cout << "  ! 警告: 解锁后肩/肘关节会因重力下坠, 请双手扶稳!\n";
    for (int mid : motor_ids) {
        auto s = decltype(ht.stop(mid)){};
        try { s = ht.stop(mid); } catch (...) {}
        if (s && s->mode == 0x00) {
            std::cout << "  电机 " << mid << ": OK (mode=0x00, 可手动转动)\n";
        } else if (s) {
            std::cout << "  电机 " << mid << ": 切换异常 (mode=0x"
                      << std::hex << s->mode << std::dec << ")\n";
        } else {
            std::cout << "  电机 " << mid << ": 无响应\n";
        }
    }
    std::cout << "  → 现在请用手把每个关节摆到目标零点姿态 (下面会读 5 次状态)\n";

    // ----- 阶段 2: 读取当前状态, 边摆边看 -----
    std::cout << "\n--- 阶段 2: 当前状态 (5 次, 用于观察手动摆位) ---\n";
    for (int i = 0; i < 5; ++i) {
        std::cout << "[" << (i + 1) << "/5]\n";
        print_states(ht, motor_ids);
        sleep_seconds(0.5);
    }

    // ----- 阶段 3: 二次确认 (输入 yes 才执行) -----
    std::cout << "\n========================== 危险操作 ==========================\n"
              << "  即将把以下 " << motor_ids.size() << " 个关节的当前位置设为新零点:\n"
              << "    电机 ID:";
    for (int mid : motor_ids) std::cout << " " << mid;
    std::cout << "\n  reset 过程中电机会短暂失力, 肩/肘等承重关节可能下坠!\n"
              << "  请双手扶稳机械臂, 然后输入 'yes' 并回车继续 (其它输入=取消):\n"
              << "  > " << std::flush;
    {
        std::string answer;
        std::getline(std::cin, answer);
        const std::string a = strip_lower(answer);
        if (a != "yes") {
            std::cout << "\n已取消, 未做任何修改.\n";
            ht.close();
            return 0;
        }
    }
    std::cout << "==============================================================\n";

    // ----- 阶段 4: 执行 reset_zero / save_config / motor_reset -----
    std::cout << "\n--- 阶段 4: 执行 reset_zero ---\n";
    const std::size_t total = motor_ids.size();
    for (std::size_t i = 0; i < total; ++i) {
        const int mid = motor_ids[i];
        std::cout << "\n[" << (i + 1) << "/" << total << "] 电机 " << mid << "\n";
        std::cout << "    reset_zero  -> " << ht.reset_zero(mid)  << "\n";
        sleep_seconds(0.5);
        std::cout << "    save_config -> " << ht.save_config(mid) << "\n";
        sleep_seconds(0.5);
        std::cout << "    motor_reset -> " << ht.motor_reset(mid) << "\n";
        // 关节间停顿: 给重力下坠的关节预留反应时间
        if (i + 1 < total) {
            std::cout << "    (停顿 " << PER_MOTOR_PAUSE << "s, 准备处理下一个关节)\n";
            sleep_seconds(PER_MOTOR_PAUSE);
        }
    }

    std::cout << "\n等待最后一个电机重启...\n";
    sleep_seconds(2.0);

    // ----- 阶段 5: 重启后再切到 free, 验证零位 -----
    // motor_reset 后 firmware 默认 mode 不固定, 这里再发一次 stop 让用户能手动检查零位
    std::cout << "\n--- 阶段 5: 切到 free + 验证零位 ---\n";
    for (int mid : motor_ids) {
        try { ht.stop(mid); } catch (...) {}
    }
    for (int i = 0; i < 10; ++i) {
        std::cout << "[" << (i + 1) << "/10]\n";
        print_states(ht, motor_ids);
        sleep_seconds(0.5);
    }

    // 退出时保持 free, 让用户能继续手动操作机械臂
    for (int mid : motor_ids) {
        try { ht.stop(mid); } catch (...) {}
    }
    ht.close();
    std::cout << "\n=== 完成 (电机保持 free 状态, 可继续手动操作) ===\n";
    return 0;
}
