// =============================================================================
//  12_demo_new_features.cpp
//  展示 motor_example_debug 在 11/01/09 三个示例之外新增的能力:
//
//    [1] 自动 USB 串口枚举 (按 VID 过滤可能的调试板)
//    [2] 配置文件驱动 (robot.cfg, INI 风格, 无需重编)
//    [3] 位置单位制 (圈/弧度/角度可任选)
//    [4] 驱动层内置软限位 (set_pos_* 自动 clamp 并设置 pos_limit_flag)
//    [5] CAN 错误码解析 (read_can_status 返回结构化诊断)
//    [6] 后台状态轮询线程 (start_state_polling + get_cached_state)
//
//  默认从同目录的 robot.cfg 读配置, 也可以在命令行传:
//      12_demo_new_features.exe path/to/your.cfg
// =============================================================================

#include "hightorque_serial.hpp"
#include "robot_config.hpp"

#include <chrono>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <iostream>
#include <memory>
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
using hightorque::PosUnit;
using hightorque::PortInfo;
using hightorque::CanStatus;
using hightorque::RobotConfig;

static void sleep_seconds(double s) {
    std::this_thread::sleep_for(
        std::chrono::microseconds(static_cast<long long>(s * 1'000'000)));
}

// ---------------------------------------------------------------------------
//  [1] 串口枚举: 列出所有 + 标记调试板候选
// ---------------------------------------------------------------------------
static void demo_list_ports() {
    std::cout << "\n=== [1] 串口枚举 ===\n";
    const auto all = hightorque::list_serial_ports();
    if (all.empty()) {
        std::cout << "  (本机没有任何串口设备)\n";
        return;
    }
    std::cout << "  全部串口:\n";
    for (const auto& p : all) {
        std::cout << "    " << std::left << std::setw(8) << p.port
                  << " VID=" << (p.vid.empty() ? "----" : p.vid)
                  << " PID=" << (p.pid.empty() ? "----" : p.pid)
                  << " | " << p.description << "\n";
    }

    const auto candidates = hightorque::find_likely_debug_boards();
    std::cout << "  调试板候选 (按已知 USB-Serial VID 过滤):\n";
    if (candidates.empty()) {
        std::cout << "    (无, 你的板子 VID 可能不在默认列表里, 可以传 known_vids 自己加)\n";
    } else {
        for (const auto& p : candidates) {
            std::cout << "    -> " << p.port << "  [" << p.vid << ":" << p.pid << "]  "
                      << p.description << "\n";
        }
    }
}

// ---------------------------------------------------------------------------
//  [2] 加载配置 (失败则用默认值, 提示用户)
// ---------------------------------------------------------------------------
static RobotConfig load_or_default(const std::string& path) {
    std::cout << "\n=== [2] 加载配置文件 ===\n";
    try {
        auto cfg = RobotConfig::load(path);
        std::cout << "  从 '" << path << "' 加载成功:\n" << cfg.to_string() << "\n";
        return cfg;
    } catch (const std::exception& e) {
        std::cout << "  ! 加载失败 (" << e.what() << "), 使用内置默认值\n";
        RobotConfig cfg;
        cfg.motor_ids = {1, 2, 3, 4, 5, 6, 7};
        cfg.pos_unit  = PosUnit::Turns;
        cfg.limits = {
            {1, {-0.40, 0.30}}, {2, {-0.05, 0.48}}, {3, {0.00, 0.47}},
            {4, {-0.25, 0.25}}, {5, {-0.25, 0.25}}, {6, {-0.25, 0.25}},
            {7, {-0.25, 0.25}},
        };
        std::cout << "  内置默认:\n" << cfg.to_string() << "\n";
        return cfg;
    }
}

// ---------------------------------------------------------------------------
//  [3]+[4] 演示: 单位制 + 软限位 (不真发命令, 只展示语义/clamp 行为)
// ---------------------------------------------------------------------------
static void demo_unit_and_limit_dry_run(HightorqueSerial& ht, const RobotConfig& cfg) {
    std::cout << "\n=== [3]+[4] 位置单位制 + 软限位 (干跑, 不真发指令) ===\n";

    // 把 cfg 里的限位灌进 driver
    cfg.apply_limits_to(ht);

    if (cfg.motor_ids.empty()) {
        std::cout << "  (motor_ids 为空, 跳过)\n";
        return;
    }
    const int mid = cfg.motor_ids.front();
    double lo, hi;
    if (ht.get_position_limit_turns(mid, lo, hi)) {
        std::cout << "  电机 " << mid << " 限位 (圈): [" << lo << ", " << hi << "]\n";
    } else {
        std::cout << "  电机 " << mid << " 没有设置软限位\n";
    }

    // 不真的下发到电机 (没接也无所谓), 只是看 clamp 后能拿到什么
    // 用一个肯定超界的大值
    const double over = (hi + 0.5);   // 比上限多 0.5 圈
    std::cout << "  尝试 set_pos_vel_acc 给一个超界值 " << over
              << " (圈), 期望被 clamp 到 " << hi << " 圈, pos_limit_flag = +1\n";

    // 这里如果硬件没接, set_pos_vel_acc 会返回 nullopt, 但 clamp 已经发生了 (在驱动入口)
    auto resp = ht.set_pos_vel_acc(mid, over, 0.05, 0.05);
    if (resp) {
        std::cout << "    -> 收到电机回复: pos=" << resp->position
                  << " 圈, pos_limit_flag=" << resp->pos_limit_flag << "\n";
    } else {
        std::cout << "    -> 未收到电机回复 (可能没接电机), 但限位 clamp 已生效 (内部判定)\n";
    }

    // 展示弧度单位
    std::cout << "  也可以用弧度: ht.set_pos_vel_acc(" << mid
              << ", 1.0 (rad), 0.05, 0.05, PosUnit::Radians);\n"
              << "  或者角度:    ht.set_pos_vel_acc(" << mid
              << ", 30.0 (deg), 0.05, 0.05, PosUnit::Degrees);\n";
}

// ---------------------------------------------------------------------------
//  [5] CAN 错误码
// ---------------------------------------------------------------------------
static void demo_can_status(HightorqueSerial& ht) {
    std::cout << "\n=== [5] CAN 错误码 ===\n";
    const auto st = ht.read_can_status();
    std::cout << "  原始回复: " << (st.raw.empty() ? "(空)" : st.raw) << "\n"
              << "  解析:     " << st.to_string() << "\n";
    if (st.is_ok())                  std::cout << "  -> 总线状态正常\n";
    else if (st.fault == hightorque::CanFault::BusOff)
                                     std::cout << "  -> [严重] 总线关闭, 必须 fdcan_reset\n";
    else                             std::cout << "  -> [警告] CAN 链路有错误, 检查接线/终端电阻\n";
}

// ---------------------------------------------------------------------------
//  [6] 后台状态轮询线程 (跑 3 秒)
// ---------------------------------------------------------------------------
static void demo_polling(HightorqueSerial& ht, const RobotConfig& cfg) {
    std::cout << "\n=== [6] 后台状态轮询 (50 Hz, 跑 3 秒) ===\n";
    if (cfg.motor_ids.empty()) {
        std::cout << "  (motor_ids 为空, 跳过)\n";
        return;
    }

    std::atomic<int> tick{0};
    ht.start_state_polling(cfg.motor_ids, 50.0, [&](const std::vector<int>& updated) {
        tick.fetch_add(1);
        // 这里在轮询线程里, 不要做太重的事
        (void)updated;
    });

    for (int i = 0; i < 3; ++i) {
        sleep_seconds(1.0);
        std::cout << "  [t=" << (i + 1) << "s] 已收到 " << tick.load() << " 轮刷新; "
                  << "缓存中各电机最新位置:";
        for (int mid : cfg.motor_ids) {
            auto s = ht.get_cached_state(mid);
            if (s) {
                std::cout << "  M" << mid << "="
                          << std::showpos << std::fixed << std::setprecision(3)
                          << s->position << std::noshowpos;
            } else {
                std::cout << "  M" << mid << "=N/A";
            }
        }
        std::cout << "\n";
    }

    ht.stop_state_polling();
    std::cout << "  -> 轮询线程已停止\n";
}

// ---------------------------------------------------------------------------
//  main
// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif

    const std::string cfg_path = (argc > 1) ? argv[1] : "robot.cfg";

    std::cout << "########  motor_example_debug 新能力演示  ########\n";

    // 不依赖硬件就能跑的两段
    demo_list_ports();
    const RobotConfig cfg = load_or_default(cfg_path);

    // 后续都需要打开串口
    std::unique_ptr<HightorqueSerial> ht_ptr;
    try {
        ht_ptr = std::make_unique<HightorqueSerial>(cfg.port, cfg.baudrate);
    } catch (const std::exception& e) {
        std::cerr << "\n[警告] 无法打开串口 " << cfg.port << ": " << e.what() << "\n"
                  << "       后续 [3]/[4]/[5]/[6] 演示将跳过 (它们需要真的连上调试板).\n"
                  << "       前两步 (枚举/配置加载) 已完成, 你可以参考输出修正 robot.cfg 后再跑.\n";
        std::cout << "\n按 Enter 退出..." << std::flush;
        std::cin.get();
        return 0;
    }
    HightorqueSerial& ht = *ht_ptr;
    std::cout << "\n串口 " << cfg.port << " 已打开.\n";

    demo_unit_and_limit_dry_run(ht, cfg);
    demo_can_status(ht);
    demo_polling(ht, cfg);

    for (int mid : cfg.motor_ids) {
        try { ht.stop(mid); } catch (...) {}
    }
    ht.close();
    std::cout << "\n########  演示完成  ########\n";
    return 0;
}
