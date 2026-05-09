# motor_example_debug — C++ 端口

调试板 USB 串口协议的 C++ 实现，1:1 移植自 `panthera_python/scripts/motor_example/`。

cd F:\1LWP\Python\Panthera_Workspace\Panthera-HT_SDK\panthera_cpp\motor_example_debug
  .\build\bin\Release\11_multi_joint_control.exe

  01_motor_get_status.exe
  09_set_zero.exe
  14_set_limits.exe

| Python 源 | C++ 端口 |
|---|---|
| `hightorque_serial.py` | `include/hightorque_serial.hpp` + `src/hightorque_serial.cpp` |
| `01_motor_get_status.py` | `src/01_motor_get_status.cpp` |
| `09_set_zero.py` | `src/09_set_zero.cpp` (扩展: 交互选择关节) |
| `11_multi_joint_control.py` | `src/11_multi_joint_control.cpp` |

⚠ **协议**：本目录走的是**调试板 (Debug Board)** 的 ASCII 命令协议 (`can send <ID> <DATA>\r\n`)，与隔壁 `motor_cpp/`、`robot_cpp/` 走的"通讯板二进制协议"**完全不同**，不能混用。

## 目录结构

```
motor_example_debug/
├── CMakeLists.txt
├── README.md
├── robot.cfg                         # 示例配置文件 (INI 风格)
├── include/
│   ├── hightorque_serial.hpp         # 驱动类型/函数声明
│   └── robot_config.hpp              # 轻量 INI 配置解析 (header-only)
└── src/
    ├── hightorque_serial.cpp         # CAN 帧构建/解析、串口收发、软限位、轮询线程
    ├── 01_motor_get_status.cpp       # 循环读状态
    ├── 09_set_zero.cpp               # 重置零点 (交互选关节)
    ├── 11_multi_joint_control.cpp    # 交互式多关节控制
    └── 12_demo_new_features.cpp      # 新能力演示
```

## 在 Python 版基础上新增的能力

| # | 能力 | 用法 |
|---|---|---|
| 1 | **位置单位制** | `ht.set_pos_vel_acc(id, 30.0, 0.05, 0.05, PosUnit::Degrees)`，可选 `Turns / Radians / Degrees` |
| 2 | **驱动层内置软限位** | `ht.enable_position_limit(id, lo, hi, unit)`；超界自动 clamp，并在返回的 `MotorState::pos_limit_flag` 置 ±1 |
| 3 | **CAN 错误码解码** | `auto st = ht.read_can_status(); if (!st.is_ok()) ...`，`st.fault` 区分 OK/Warning/Passive/BusOff |
| 4 | **USB 串口枚举** | `auto cands = find_likely_debug_boards();`（按 ST/FTDI/CH340/SiLabs/Prolific VID 过滤） |
| 5 | **INI 配置文件** | `auto cfg = RobotConfig::load("robot.cfg"); cfg.apply_limits_to(ht);` |
| 6 | **后台状态轮询线程** | `ht.start_state_polling({1,2,3}, 50.0); auto s = ht.get_cached_state(2);` |

> **零新依赖** — 全部用 C++17 标准库 + 现有的 `serial_cmake`，没有 yaml-cpp / ROS / lcm。

## 依赖

| 依赖 | 来源 |
|---|---|
| C++17 | MSVC 2017+ / GCC 7+ / Clang 5+ |
| `serial_cmake` (跨平台串口库) | 直接复用 SDK 自带的 `panthera_cpp/motor_cpp/third_part/serial_cmake/` |
| Win32 控制台 (`<conio.h>`) | Windows 自带 |
| POSIX (`<termios.h>`, `select`) | Linux 自带 |

> 不需要装 ROS、yaml-cpp、lcm、libserialport 等。

## 构建

### Windows (MSVC, 推荐)

```bat
cd Panthera-HT_SDK\panthera_cpp\motor_example_debug
cmake -S . -B build
cmake --build build --config Release
```

可执行文件：`build\bin\Release\11_multi_joint_control.exe`
（`serial_cmake.dll` 会自动复制到 exe 同目录。）

### Linux / WSL

```bash
cd Panthera-HT_SDK/panthera_cpp/motor_example_debug
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

可执行文件：`build/bin/11_multi_joint_control`

## 使用

```
build\bin\Release\11_multi_joint_control.exe
```

交互界面与 Python 版一致：

```
> 1 0.1                # 1 号电机转到 0.1 圈
> all 0 0 0 0 0 0 0    # 七个关节同时回到 0
> home                 # 等同上面那行
> q                    # 退出
Ctrl+Q                 # 任何时刻紧急停止 (运动中 / 输入中均有效)
```

## 修改硬件参数

`src/11_multi_joint_control.cpp` 顶部：

```cpp
static const std::string PORT     = "COM14";
static const uint32_t    BAUDRATE = 4'000'000u;
static const std::vector<int> MOTOR_IDS = {1, 2, 3, 4, 5, 6, 7};
static const std::map<int, std::pair<double, double>> JOINT_LIMITS = {
    {1, {-0.40, 0.30}},   // 底座
    ...
};
```

改完重新 `cmake --build build --config Release` 即可。

## 与 Python 版的对应关系

| Python | C++ |
|---|---|
| `from hightorque_serial import HightorqueSerial` | `#include "hightorque_serial.hpp"` |
| `ht = HightorqueSerial(PORT, BAUDRATE)` | `hightorque::HightorqueSerial ht(PORT, BAUDRATE);` |
| `ht.set_pos_vel_acc(mid, pos, vel, acc)` | `ht.set_pos_vel_acc(mid, pos, vel, acc)` (返回 `std::optional<MotorState>`) |
| `s.position` (float) | `s->position` (double) |
| `MotorState | None` | `std::optional<MotorState>` |
| `dict[int, float]` | `std::map<int, double>` |
| `EmergencyStop` exception | `EmergencyStop` (继承 `std::exception`) |
| `msvcrt.kbhit/getch` | `_kbhit/_getch` (`<conio.h>`) |
| `termios + select` | `termios + select` (POSIX, 同名接口) |

## 安全提醒

- 首次运行先把 `MOTOR_IDS` 改成 `{1}` 单关节测试。
- `JOINT_LIMITS` 中 4~7 号是**保守初值**，必须按你机械臂的实际行程修改。
- 电机失控时直接拔掉调试板 USB 线。
- `stop()` 只是关 PWM，**重力下电机仍会下坠**——肩/肘等承重关节请额外注意。
