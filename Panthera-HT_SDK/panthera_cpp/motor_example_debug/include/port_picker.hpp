// =============================================================================
//  port_picker.hpp
//  USB 串口自动识别工具 (header-only).
//
//  策略:
//    1) 列出系统所有串口 (serial::list_ports())
//    2) 用 hardware_id / description 字符串过滤 USB 串口 (排除蓝牙/虚拟端口)
//    3) 显示候选列表给用户参考
//    4) 选择规则:
//         a. preferred 在候选里 → 用 preferred
//         b. 仅 1 个 USB 候选 → 自动用它
//         c. 多个候选 → 默认用第 1 个 (提示用户可指定)
//    5) 没有任何 USB 串口 → 返回 preferred 让上层 try (并报错)
//
//  preferred 参数特殊值:
//    "auto" / ""      → 跳过偏好, 直接走自动选择
//    "COM14" 等具体值 → 优先尝试, 不存在 fallback 自动选
//
//  使用:
//    #include "port_picker.hpp"
//    std::string port = hightorque::pick_serial_port("auto");
//    HightorqueSerial ht(port, 4'000'000u);
// =============================================================================
#pragma once

#include <serial/serial.h>

#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace hightorque {

inline bool is_usb_port(const serial::PortInfo& p) {
    auto contains_usb = [](const std::string& s) {
        // 大小写不敏感匹配 "USB"
        for (std::size_t i = 0; i + 3 <= s.size(); ++i) {
            if ((s[i]   == 'u' || s[i]   == 'U')
             && (s[i+1] == 's' || s[i+1] == 'S')
             && (s[i+2] == 'b' || s[i+2] == 'B')) return true;
        }
        return false;
    };
    return contains_usb(p.hardware_id) || contains_usb(p.description);
}

// 返回选中的端口名 (空字符串 = 没找到 / 用户放弃, 让上层报错处理)
inline std::string pick_serial_port(const std::string& preferred) {
    std::vector<serial::PortInfo> all_ports;
    try {
        all_ports = serial::list_ports();
    } catch (const std::exception& e) {
        std::cout << "[端口] 枚举失败: " << e.what() << "\n";
        return preferred;
    }

    std::vector<serial::PortInfo> usb_ports;
    for (const auto& p : all_ports) {
        if (is_usb_port(p)) usb_ports.push_back(p);
    }

    std::cout << "[端口] 系统检测到 " << all_ports.size() << " 个串口, "
              << usb_ports.size() << " 个是 USB:\n";
    for (const auto& p : usb_ports) {
        const bool is_pref = (!preferred.empty() && p.port == preferred);
        std::cout << "    " << (is_pref ? "★ " : "  ")
                  << std::setw(10) << std::left << p.port << std::right
                  << "  " << p.description
                  << "  [" << p.hardware_id << "]\n";
    }

    if (usb_ports.empty()) {
        std::cout << "[端口] 未找到任何 USB 串口. 请检查调试板 USB 是否插好.\n";
        return preferred;
    }

    // 偏好端口在候选里 → 直接用
    const bool use_preferred = !preferred.empty() && preferred != "auto";
    if (use_preferred) {
        for (const auto& p : usb_ports) {
            if (p.port == preferred) {
                std::cout << "[端口] 使用指定的 " << preferred << "\n";
                return preferred;
            }
        }
        std::cout << "[端口] 指定的 '" << preferred
                  << "' 不在 USB 列表中, 改为自动选择\n";
    }

    // 唯一一个 USB → 自动用
    if (usb_ports.size() == 1) {
        std::cout << "[端口] 唯一 USB 端口: " << usb_ports[0].port
                  << " (自动选择)\n";
        return usb_ports[0].port;
    }

    // 多个 → 默认用第一个
    std::cout << "[端口] 多个 USB 端口, 默认使用第一个: " << usb_ports[0].port
              << " (如需指定, 命令行加端口名作为参数)\n";
    return usb_ports[0].port;
}

} // namespace hightorque
