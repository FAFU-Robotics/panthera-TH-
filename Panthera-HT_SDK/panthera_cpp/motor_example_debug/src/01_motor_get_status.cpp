// =============================================================================
//  01_motor_get_status.cpp
//  循环读取并打印所有电机状态 — C++ 端口
//
//  快捷键:
//    Ctrl+C  → 退出程序
//
//  行为:
//    - 启动时把所有电机切到 mode=0x00 (free), 关节可以用手随意转动
//    - 之后只读状态, 不发位置控制指令
//    - 退出时再发一次 stop, 确保关节继续保持自由 (避免上一个程序留下的锁定)
//
//  机械臂安全提醒: 本脚本不会主动控制电机.
//    - 但若上一个程序刚好把肩/肘 (2/3 号) 关节切到位置模式且断电后没复位,
//      本程序启动瞬间把它切到 free 时, 关节会因重力下坠 — 请扶稳.
// =============================================================================

#include "hightorque_serial.hpp"
#include "port_picker.hpp"     // hightorque::pick_serial_port() — USB 自动识别

#include <atomic>
#include <chrono>
#include <csignal>
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

// ---------------------------------------------------------------------------
//  配置
//
//  PORT_PREFERRED 用 "auto" 时, 程序自动从系统 USB 串口里选一个;
//  也可以传命令行参数指定具体端口名 (例: 01_motor_get_status.exe COM14).
// ---------------------------------------------------------------------------

static const std::string PORT_PREFERRED = "auto";
static const uint32_t    BAUDRATE       = 4'000'000u;
static const std::vector<int> MOTOR_IDS = {1, 2, 3, 4, 5, 6, 7};

// ---------------------------------------------------------------------------
//  Ctrl+C 信号处理
//
//  双保险:
//    1) std::signal(SIGINT) — POSIX/MSVC 通用
//    2) Windows: SetConsoleCtrlHandler — 处理控制台 Ctrl+C/Ctrl+Break/关闭按钮,
//       即使主线程正卡在 read() / sleep() 也能立刻把 g_exit_flag 置 true.
// ---------------------------------------------------------------------------

static std::atomic<bool> g_exit_flag{false};

extern "C" void on_signal(int) {
    g_exit_flag.store(true);
}

#ifdef _WIN32
static BOOL WINAPI on_console_ctrl(DWORD ctrl_type) {
    if (ctrl_type == CTRL_C_EVENT
     || ctrl_type == CTRL_BREAK_EVENT
     || ctrl_type == CTRL_CLOSE_EVENT
     || ctrl_type == CTRL_SHUTDOWN_EVENT) {
        g_exit_flag.store(true);
        return TRUE;
    }
    return FALSE;
}
#endif

// 可被 Ctrl+C 中断的睡眠 (步长 50ms 检查一次)
static void sleep_interruptible(double seconds) {
    using namespace std::chrono;
    const auto end = steady_clock::now() + microseconds(static_cast<long long>(seconds * 1e6));
    while (!g_exit_flag.load()) {
        const auto now = steady_clock::now();
        if (now >= end) return;
        const auto remain = duration_cast<microseconds>(end - now).count();
        std::this_thread::sleep_for(microseconds(std::min<long long>(50'000, remain)));
    }
}

int main(int argc, char** argv) {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCtrlHandler(on_console_ctrl, TRUE);
#endif
    std::signal(SIGINT, on_signal);

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
                  << "       命令行可指定端口: 01_motor_get_status.exe COM14\n";
        std::cout << "\n按 Enter 退出..." << std::flush;
        std::cin.get();
        return 1;
    }
    HightorqueSerial& ht = *ht_ptr;

    // ---- 切所有电机到 mode=0x00 (free), 让关节可以手动转动 ----
    // 注意: 上一个程序可能把电机锁在 mode=10/0x0F, 现在解锁它们.
    // 重力关节 (2/3 号) 在解锁瞬间会下坠 — 请扶稳!
    std::cout << "切电机到 free 模式 (mode=0x00, 可手动转动)...\n";
    for (int mid : MOTOR_IDS) {
        if (g_exit_flag.load()) break;
        auto s = decltype(ht.stop(mid)){};
        try { s = ht.stop(mid); } catch (...) {}
        if (s && s->mode == 0x00) {
            std::cout << "  电机 " << mid << ": OK (mode=0x00)\n";
        } else if (s) {
            std::cout << "  电机 " << mid << ": 切换异常 (mode=0x"
                      << std::hex << s->mode << std::dec << ")\n";
        } else {
            std::cout << "  电机 " << mid << ": 无响应\n";
        }
    }
    std::cout << "\n读取 " << MOTOR_IDS.size()
              << " 个电机状态 (Ctrl+C 退出, 关节可手动转动)\n\n";

    while (!g_exit_flag.load()) {
        for (int mid : MOTOR_IDS) {
            if (g_exit_flag.load()) break;

            // 短超时 (0.2s) 让 Ctrl+C 响应更快; 在线电机一般 1~2ms 就回包
            const auto state = ht.read_motor_state(mid, 0.2);
            if (!state) {
                std::cout << "电机 " << mid << ": 无响应\n";
                continue;
            }
            // 协议原生单位是"圈" / "转/秒", 这里乘 360 显示为"度" / "度/秒"
            const double pos_deg = state->position * 360.0;
            const double vel_dps = state->velocity * 360.0;
            std::cout << "电机 " << mid << "  "
                      << "位置: " << std::showpos << std::fixed << std::setprecision(2)
                                  << std::setw(8) << pos_deg << std::noshowpos << " 度  "
                      << "速度: " << std::showpos << std::fixed << std::setprecision(2)
                                  << std::setw(8) << vel_dps << std::noshowpos << " 度/秒  "
                      << "力矩: " << std::showpos << std::setw(6)
                                  << static_cast<int>(state->torque) << std::noshowpos << " (raw)  "
                      << "模式: " << state->mode << "  "
                      << "故障: 0x" << std::hex << std::uppercase << std::setw(2)
                                    << std::setfill('0') << state->fault
                                    << std::dec << std::setfill(' ') << "\n";
        }
        if (g_exit_flag.load()) break;

        std::cout << std::string(72, '-') << "\n";
        sleep_interruptible(0.5);
    }

    std::cout << "\n已退出, 保持电机为 free 状态 (mode=0x00)\n";

    // 退出时再发一次 stop, 确保关节继续可手动转动
    for (int mid : MOTOR_IDS) {
        try { ht.stop(mid); } catch (...) {}
    }
    ht.close();
    return 0;
}
