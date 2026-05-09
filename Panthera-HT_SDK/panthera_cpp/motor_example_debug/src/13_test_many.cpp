// =============================================================================
//  13_test_many.cpp
//
//  一拖多 (CAN ID 0x8090) 实测验证 + 性能对比.
//
//  用法:
//      13_test_many.exe                    # 默认读 robot.cfg, 不动电机, 只发 stop+查询
//      13_test_many.exe --move             # 真正驱动电机做小范围回零运动 (危险, 需通电)
//      13_test_many.exe my-other.cfg       # 指定配置
//      13_test_many.exe --move my-other.cfg
//
//  3 种发送方式对比 100 次的总耗时 / 单次平均:
//      A. 串行单发: for id in motors: ht.read_motor_state(id)
//      B. 后台轮询: ht.start_state_polling(...) + ht.get_cached_state(id)
//      C. 一拖多 :  ht.set_many_pos_vel_tqe([{id, pos=0, vel=0, tqe=0}, ...])
//
//  注意:
//    - 只有 --move 模式会下发非零的 pos/vel/tqe, 否则 C 路径里所有 motor 槽位
//      也是 0/0/0 (= "回零保持力矩 0"); 静止电机这是无操作, 安全.
//    - --move 模式仍然只发 pos=0 (回零) + vel_max=0.05 + tqe=0, 极慢.
// =============================================================================

#include "hightorque_serial.hpp"
#include "robot_config.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <exception>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <numeric>
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
using hightorque::MotorState;
using hightorque::RobotConfig;
using hightorque::PosUnit;

namespace {
constexpr int    BENCH_ROUNDS = 100;
constexpr double TEST_VEL     = 0.05;     // 转/秒
constexpr int    TEST_TQE_RAW = 0;        // 0 = 不限力矩 (协议默认)

std::atomic<bool> g_stop{false};
void on_signal(int) { g_stop.store(true); }

double now_ms() {
    using namespace std::chrono;
    return duration<double, std::milli>(steady_clock::now().time_since_epoch()).count();
}

struct BenchResult {
    std::string         label;
    std::vector<double> per_call_ms;
    int                 success_count = 0;
    int                 total_calls   = 0;

    double avg_ms() const {
        if (per_call_ms.empty()) return 0.0;
        double s = std::accumulate(per_call_ms.begin(), per_call_ms.end(), 0.0);
        return s / per_call_ms.size();
    }
    double min_ms() const {
        return per_call_ms.empty() ? 0.0
            : *std::min_element(per_call_ms.begin(), per_call_ms.end());
    }
    double max_ms() const {
        return per_call_ms.empty() ? 0.0
            : *std::max_element(per_call_ms.begin(), per_call_ms.end());
    }
    double p99_ms() const {
        if (per_call_ms.empty()) return 0.0;
        auto sorted = per_call_ms;
        std::sort(sorted.begin(), sorted.end());
        const std::size_t idx = std::min(sorted.size() - 1,
            static_cast<std::size_t>(sorted.size() * 0.99));
        return sorted[idx];
    }
};

void print_result(const BenchResult& r, int n_motors) {
    std::cout << "  " << std::left << std::setw(20) << r.label
              << "  avg=" << std::fixed << std::setprecision(2) << std::setw(7) << r.avg_ms() << " ms"
              << "  min=" << std::setw(6) << r.min_ms() << " ms"
              << "  max=" << std::setw(6) << r.max_ms() << " ms"
              << "  p99=" << std::setw(6) << r.p99_ms() << " ms"
              << "  ok="  << r.success_count << "/" << r.total_calls
              << "  (" << n_motors << " motors)\n";
}

// 派生指标: 等价控制频率 (1 / avg_ms * 1000)
double equiv_hz(const BenchResult& r) {
    if (r.avg_ms() <= 0) return 0.0;
    return 1000.0 / r.avg_ms();
}
} // namespace

int main(int argc, char** argv) {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif
    std::signal(SIGINT, on_signal);

    bool        do_move  = false;
    bool        do_debug = false;
    std::string cfg_path = "robot.cfg";
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if      (a == "--move")  do_move  = true;
        else if (a == "--debug") do_debug = true;
        else                     cfg_path = a;
    }

    // ---- 配置 ----
    RobotConfig cfg;
    try {
        cfg = RobotConfig::load(cfg_path);
        std::cout << "[配置] 从 '" << cfg_path << "' 加载 ("
                  << cfg.motor_ids.size() << " motors @ " << cfg.port << ")\n";
    } catch (const std::exception& e) {
        std::cerr << "[配置] 无法加载 '" << cfg_path << "': " << e.what() << "\n";
        return 1;
    }
    if (cfg.motor_ids.empty()) {
        std::cerr << "[错误] motor_ids 为空, 退出.\n";
        return 1;
    }

    // ---- 串口 ----
    std::unique_ptr<HightorqueSerial> ht_ptr;
    try {
        ht_ptr = std::make_unique<HightorqueSerial>(cfg.port, cfg.baudrate);
    } catch (const std::exception& e) {
        std::cerr << "\n[错误] 无法打开 " << cfg.port << ": " << e.what() << "\n";
        return 1;
    }
    HightorqueSerial& ht = *ht_ptr;
    cfg.apply_limits_to(ht);

    const int n_motors  = static_cast<int>(cfg.motor_ids.size());
    const int max_id    = *std::max_element(cfg.motor_ids.begin(), cfg.motor_ids.end());

    std::cout << "\n========================================================\n"
              << "  一拖多 vs 单发 性能对比测试 (" << BENCH_ROUNDS << " 轮)\n"
              << "  电机: [";
    for (std::size_t i = 0; i < cfg.motor_ids.size(); ++i) {
        if (i) std::cout << ", ";
        std::cout << cfg.motor_ids[i];
    }
    std::cout << "]\n"
              << "  最大 ID: " << max_id << " (一拖多帧槽位数)\n"
              << "  --move:  " << (do_move ? "是 (会驱动电机)" : "否 (只查询/stop)") << "\n"
              << "  --debug: " << (do_debug ? "是 (打印调试板所有非 rcv 行)" : "否") << "\n"
              << "========================================================\n";

    // ---- Step 0a: 通信预检 ----
    std::cout << "\n--- 通信预检 ---\n";
    int alive = 0;
    for (int mid : cfg.motor_ids) {
        if (auto s = ht.read_motor_state(mid)) {
            std::cout << "  电机 " << mid << ": OK pos="
                      << std::showpos << std::fixed << std::setprecision(4) << s->position
                      << std::noshowpos << "\n";
            ++alive;
        } else {
            std::cout << "  电机 " << mid << ": 无响应\n";
        }
    }
    if (alive == 0) {
        std::cerr << "\n[错误] 没有任何电机响应, 退出.\n";
        return 1;
    }

    // ---- Step 0b: 固件版本 (诊断 0x8090 兼容性的关键) ----
    std::cout << "\n--- 电机固件版本 (一拖多需要较新版本; 一般 v3.0+) ---\n";
    for (int mid : cfg.motor_ids) {
        try {
            auto v = ht.read_version(mid);
            std::cout << "  电机 " << mid << ": "
                      << (v ? *v : "未读到 (老固件可能不支持 read_version)") << "\n";
        } catch (const std::exception& e) {
            std::cout << "  电机 " << mid << ": 异常 " << e.what() << "\n";
        }
    }

    // ---- Step 0c: 调试板状态 ----
    std::cout << "\n--- 调试板状态 ---\n";
    try {
        auto cs = ht.read_can_status();
        std::cout << "  " << cs.to_string() << "\n";
        if (!cs.raw.empty()) std::cout << "  原始: " << cs.raw << "\n";
    } catch (const std::exception& e) {
        std::cout << "  无法读取: " << e.what() << "\n";
    }

    // ---- A: 串行单发 read_motor_state ----
    BenchResult r_serial{"A. 串行单发 read"};
    std::cout << "\n[A] 串行单发 (for id: read_motor_state)\n";
    for (int round = 0; round < BENCH_ROUNDS; ++round) {
        if (g_stop.load()) break;
        const double t0 = now_ms();
        int got = 0;
        for (int mid : cfg.motor_ids) {
            if (auto s = ht.read_motor_state(mid, 0.05)) ++got;
        }
        const double dt = now_ms() - t0;
        r_serial.per_call_ms.push_back(dt);
        r_serial.total_calls++;
        if (got == n_motors) r_serial.success_count++;
        if ((round + 1) % 10 == 0) {
            std::cout << "    " << (round + 1) << "/" << BENCH_ROUNDS
                      << "  last=" << std::fixed << std::setprecision(2) << dt << " ms\n";
        }
    }

    // ---- B: 后台轮询 + get_cached_state ----
    BenchResult r_poll{"B. 后台轮询 cache"};
    std::cout << "\n[B] 后台轮询线程 + get_cached_state (轮询率 100Hz)\n";
    ht.start_state_polling(cfg.motor_ids, 100.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));   // 给后台线程预热
    for (int round = 0; round < BENCH_ROUNDS; ++round) {
        if (g_stop.load()) break;
        const double t0 = now_ms();
        int got = 0;
        for (int mid : cfg.motor_ids) {
            if (auto s = ht.get_cached_state(mid)) ++got;
        }
        const double dt = now_ms() - t0;
        r_poll.per_call_ms.push_back(dt);
        r_poll.total_calls++;
        if (got == n_motors) r_poll.success_count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ht.stop_state_polling();

    // ---- C: 一拖多 set_many_pos_vel_tqe ----
    BenchResult r_many{"C. 一拖多 0x8090"};
    std::cout << "\n[C] 一拖多单帧 (set_many_pos_vel_tqe -> CAN 0x8090)\n";
    if (!do_move) {
        std::cout << "    (--move 未启用, 所有槽位 pos=vel=tqe=0, 电机不动)\n";
    } else {
        std::cout << "    --move 启用: pos=0 (回零), vel_max=" << TEST_VEL << " 转/秒\n";
    }

    // 安装 debug hook 抓取调试板的所有非 rcv 行 (诊断 ack/err)
    std::vector<std::string> dbg_lines;
    std::mutex               dbg_mtx;
    if (do_debug) {
        ht.set_debug_line_handler([&](const std::string& s) {
            std::lock_guard<std::mutex> lk(dbg_mtx);
            if (dbg_lines.size() < 200) dbg_lines.push_back(s);   // 别让爆量
        });
        std::cout << "    [debug] 已安装非 rcv 行 hook, 末尾汇报\n";
    }

    for (int round = 0; round < BENCH_ROUNDS; ++round) {
        if (g_stop.load()) break;

        std::vector<HightorqueSerial::ManyMotorCmd> cmds;
        cmds.reserve(cfg.motor_ids.size());
        for (int mid : cfg.motor_ids) {
            cmds.push_back({mid,
                            do_move ? 0.0 : 0.0,         // pos = 0 (回零)
                            do_move ? TEST_VEL : 0.0,    // vel = 0 时电机不响应运动
                            TEST_TQE_RAW});
        }

        const double t0 = now_ms();
        auto states = ht.set_many_pos_vel_tqe(cmds, PosUnit::Turns, max_id, 0.05);
        const double dt = now_ms() - t0;

        r_many.per_call_ms.push_back(dt);
        r_many.total_calls++;
        if (static_cast<int>(states.size()) == n_motors) r_many.success_count++;

        if ((round + 1) % 10 == 0) {
            std::cout << "    " << (round + 1) << "/" << BENCH_ROUNDS
                      << "  last=" << std::fixed << std::setprecision(2) << dt
                      << " ms  rcv=" << states.size() << "/" << n_motors << "\n";
        }
    }

    // 收尾: 全部 stop, 别让电机带着上次指令走
    if (do_move) {
        for (int mid : cfg.motor_ids) {
            try { ht.stop(mid); } catch (...) {}
        }
    }

    // ---- 报告 ----
    std::cout << "\n========================================================\n"
              << "  实测报告 (单位: ms, 越小越好)\n"
              << "========================================================\n";
    print_result(r_serial, n_motors);
    print_result(r_poll,   n_motors);
    print_result(r_many,   n_motors);

    std::cout << "\n  等价控制频率上限:\n"
              << "    A 串行单发: " << std::fixed << std::setprecision(1)
              << equiv_hz(r_serial) << " Hz\n"
              << "    B 缓存读取: " << equiv_hz(r_poll)
              << " Hz  (注: 后台线程刷新率才是真正吞吐)\n"
              << "    C 一拖多  : " << equiv_hz(r_many)  << " Hz\n";

    if (r_many.avg_ms() > 0 && r_serial.avg_ms() > 0) {
        std::cout << "\n  C vs A 提速: "
                  << std::fixed << std::setprecision(2)
                  << (r_serial.avg_ms() / r_many.avg_ms()) << "×\n";
    }

    // ---- D: 异步 RX 模式下再发一次, 看 cache 是否被一拖多回包填充 ----
    std::cout << "\n[D] 异步 RX 验证 (启动 RX 线程, 看一拖多是否能填 cache)\n";
    ht.enable_async_rx();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ht.reset_stats();

    std::vector<HightorqueSerial::ManyMotorCmd> dummy;
    for (int mid : cfg.motor_ids) dummy.push_back({mid, 0.0, 0.0, 0});
    for (int i = 0; i < 20; ++i) {
        ht.set_many_pos_vel_tqe(dummy, PosUnit::Turns, max_id, 0.005);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));   // 让 RX 线程消化完
    auto stats_after = ht.get_stats();
    std::cout << "    " << stats_after.to_string() << "\n";

    int cached_count = 0;
    for (int mid : cfg.motor_ids) {
        if (ht.get_state(mid)) ++cached_count;
    }
    std::cout << "    cache 中有效电机数: " << cached_count << "/" << n_motors << "\n";

    ht.disable_async_rx();

    // ---- 报告 ----
    if (do_debug && !dbg_lines.empty()) {
        std::cout << "\n--- [debug] 调试板非 rcv 行 (前 20 条) ---\n";
        std::lock_guard<std::mutex> lk(dbg_mtx);
        for (std::size_t i = 0; i < std::min<std::size_t>(20, dbg_lines.size()); ++i) {
            std::cout << "    | " << dbg_lines[i] << "\n";
        }
        std::cout << "  共 " << dbg_lines.size() << " 行 (上限 200)\n";
    }

    if (r_many.success_count == 0 && stats_after.rx_parsed == 0) {
        std::cout << "\n[!] 一拖多 0 回包 — 排查清单:\n"
                  << "    (1) 上面 [Step 0b] 电机固件版本: 需要支持广播 ID 0x8090 (一般 v3.0+)\n"
                  << "    (2) 上面 [Step 0c] 调试板状态: 看是否有 bus 错误\n"
                  << "    (3) 加 --debug 重新跑, 看调试板回的非 rcv 行是不是 'err' 或别的\n"
                  << "    (4) 用 11_multi_joint_control.exe 跑一下, async 模式 warmup 后看 stats\n";
        return 2;
    }

    if (stats_after.rx_parsed > 0 && r_many.success_count == 0) {
        std::cout << "\n[*] 同步路径 0 回包 但 异步路径有回包 → 调试板 ASCII 行序异常\n"
                  << "    继续推进异步 RX 模式即可 (11 已经默认走异步).\n";
        return 0;
    }

    std::cout << "\n[OK] 一拖多协议验证通过. 可以推进到异步 RX 阶段.\n";
    return 0;
}
