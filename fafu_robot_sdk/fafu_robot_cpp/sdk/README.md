# fafu_robot_sdk (原生 C++) — `sdk/`

Fafu 机械臂的 **原生 C++ SDK**, 从 `fafu_robot_python/fafu_robot_controller.py`
1:1 移植过来, 代码风格借鉴 `Panthera-HT_SDK/.../panthera/Panthera.cpp`.

> 范围: **关节空间** 控制 + 夹爪 (含力控抓取). **不包含** 笛卡尔运动
> (move_p / move_l)、URDF / IK / Pinocchio 等. 笛卡尔规划留给 Python 侧
> (用 wrs 等库做完之后, 把关节路径喂给本 SDK 执行).

---

## 目录

```
sdk/
├── README.md                                       ← 你正在看的文件
├── CMakeLists.txt                                  ← 由父 CMakeLists 通过 add_subdirectory(sdk) 引入
│
├── include/
│   └── fafu/
│       └── fafu_robot_controller.hpp               ← 唯一公开头, 用户写 `#include "fafu/fafu_robot_controller.hpp"`
│
├── src/
│   └── fafu_robot_controller.cpp                   ← 实现 (依赖 ../include/hightorque_serial.hpp + robot_config.hpp)
│
└── examples/                                       ← 三个独立 .exe, 演示典型用法
    ├── 01_smoke.cpp        连接 / 列电机 / 读一次状态 / 干净断开 (不上电, 安全)
    ├── 02_move_j.cpp       go_home + 小幅多关节运动                 (会让机械臂运动)
    └── 03_gripper.cpp      open / close / grasp / release           (会让夹爪闭合)
```

构建产物 (Release, Windows):

```
build/sdk/Release/fafu_robot_sdk.lib                ← 静态库, 上层可以链接它做自己的控制程序
build/bin/Release/01_smoke.exe                       ← + robot.cfg + serial_cmake.dll
build/bin/Release/02_move_j.exe                      ← + robot.cfg + serial_cmake.dll
build/bin/Release/03_gripper.exe                     ← + robot.cfg + serial_cmake.dll
```

---

## 怎么 build

跟 pybind11 模块共用顶层 `fafu_robot_cpp/build.bat`. 一行命令编全部:

```bat
cd fafu_robot_sdk\fafu_robot_cpp
build.bat
```

只想编 SDK / 不要 pybind11 (不依赖 Python 也能编):

```bash
cd fafu_robot_sdk/fafu_robot_cpp
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release ^
      -DFAFU_BUILD_PYBIND11_MODULE=OFF ^
      -DFAFU_BUILD_NATIVE_SDK=ON ^
      -DFAFU_BUILD_SDK_EXAMPLES=ON
cmake --build build --config Release -j
```

可选 CMake 选项:

| 选项 | 默认 | 含义 |
|---|---|---|
| `FAFU_BUILD_PYBIND11_MODULE` | ON | 是否编 `panthera_motor.pyd` (依赖 pybind11) |
| `FAFU_BUILD_NATIVE_SDK`      | ON | 是否编 `fafu_robot_sdk.lib` |
| `FAFU_BUILD_SDK_EXAMPLES`    | ON | 是否编 `01_smoke / 02_move_j / 03_gripper` 例程 |

---

## API 速览

完整签名见 [`include/fafu/fafu_robot_controller.hpp`](include/fafu/fafu_robot_controller.hpp).
跟 Python 侧 `fafu_robot_controller.py` 一一对应:

| 类别 | C++                                                | Python                          |
|---|---|---|
| 构造  | `FafuRobotController(cfg_path, Options)`             | `FafuRobotController(cfg_path, **kwargs)` |
| 电源  | `enable / disable / brake`                           | 同名                            |
| 关节  | `move_j(angles, MoveOpts)`                           | `move_j(angles, speed=, block=, is_radians=, tolerance=)` |
|      | `go_home(speed, block)`                              | 同名                            |
| 状态  | `get_joint_values(prefer_cache)`                     | 同名                            |
|      | `get_joint_velocities(prefer_cache)`                 | 同名                            |
|      | `get_motor_states(prefer_cache)`                     | 同名                            |
| 夹爪  | `gripper_control(angle, GripperOpts) -> GraspResult?`| 同名 (signature 一致)           |
|      | `open_gripper / close_gripper / release`             | 同名                            |
|      | `grasp(GraspOpts) -> GraspResult`                    | 同名                            |
|      | `get_gripper_state() -> MotorState`                  | 同名                            |
| 限位  | `set_limit / get_limit / disable_limit / clear_limits`| 同名                           |
| 急停  | `emergency_stop / resume`                            | 同名                            |
| 杂项  | `get_status / get_can_status / reset_zero(id, confirm)` | 同名                         |
| 关闭  | `close_connection(joint_release, gripper_release)`   | 同名 (枚举 vs 字符串)           |

C++ 的 `MoveOpts / GripperOpts / GraspOpts` struct 等价于 Python 的关键字
参数, 让函数签名更短.

---

## 最小用例

```cpp
#include "fafu/fafu_robot_controller.hpp"
#include <iostream>

int main() {
    using namespace fafu_robot;

    FafuRobotController::Options opts;
    opts.has_gripper      = true;
    opts.gripper_motor_id = 7;
    // opts.port / opts.baudrate 都留默认, 用 cfg 里的

    FafuRobotController arm("robot.cfg", opts);

    arm.go_home(/*speed=*/15);              // 阻塞 S-curve 归零

    arm.open_gripper();                     // 走到夹爪软限位上限
    auto r = arm.grasp({.force_threshold = 500, .vel = 0.15});
    if (r.grasped) std::cout << "got it: " << r.to_string() << "\n";
    else           std::cout << "miss: "   << r.reason     << "\n";

    arm.release();
    // 析构里自动 close_connection(joints=Stop, gripper=Brake)
    return 0;
}
```

链接:

```cmake
add_executable(my_app main.cpp)
target_link_libraries(my_app PRIVATE fafu_robot_sdk)
```

只要 `fafu_robot_sdk/` 在你的 build 树里, 父 CMakeLists 已经把它的 include
路径用 `PUBLIC` 暴露出来了, 你直接 link 就能 include.

---

## 单位 / 约定

- **关节角**: 默认 **弧度** (`is_radians = true`); `degree` 模式传 `false`
  即可.
- **角速度**: `get_joint_velocities()` 返回 rad/s.
- **夹爪 angle**: 跟关节同步, 默认弧度. Fafu 夹爪是 *角度越大越开*
  (软限位上限 = 张开).
- **力矩 (raw)**: int16, 跟 `MotorState.torque` 同步. `GraspResult.peak_torque_raw`
  也是这个单位. 标定方法见 `fafu_robot_python/tests/test_fafu_grasp_calibrate.py`
  (Python 侧的标定脚本, 标出来的阈值可以直接拿到 C++ 这边用).
- **闭合速度**: turns/s. 默认 0.15 (~ 54 deg/s, 比 open_gripper 慢, 接触更柔).

---

## 跟 Python 侧的关系

```
                            (用户代码)
                                 │
              ┌──────────────────┼──────────────────┐
              ▼                                     ▼
   Python: FafuRobotController        C++: fafu_robot::FafuRobotController
       (fafu_robot_python/)              (fafu_robot_cpp/sdk/)
              │                                     │
              │   import panthera_motor             │   #include "fafu/..."
              ▼                                     ▼
       panthera_motor.pyd                   fafu_robot_sdk.lib
        (pybind11)                          (C++ 静态库)
              │                                     │
              └──────────────┬──────────────────────┘
                             ▼
                hightorque::HightorqueSerial   (调试板协议层, 同一份 vendored 源码)
                             │
                             ▼
                  serial_cmake (.dll)         (跨平台串口库, 同一份)
                             │
                             ▼
                          USB / 调试板 / 电机
```

两侧调用同一个底层 `HightorqueSerial`, 所以**软限位 / 状态缓存 / 异步 RX**
等行为在两侧 100% 一致. 你可以混用 (例如用 Python 做高层规划, 把关键
关节路径段交给 C++ 程序实时跑) 而不用担心 fight 同一根串口.

> 不过同一台机器上 **同一时刻** 只能有一个进程占串口. 如果 Python 在跑,
> C++ exe 启动会 fail (port busy), 反之亦然.

---

## 排错

| 现象 | 处理 |
|---|---|
| `[ERROR] 加载配置失败 robot.cfg: 无法打开配置文件` | exe 工作目录里没 cfg. POST_BUILD 已经 copy 到了 exe 同目录, cd 进去再跑 |
| `[ERROR] auto: 未找到候选 USB 调试板, 检查 USB / 驱动` | 没插调试板 / 驱动没装 / VID 不在白名单. 把 cfg 里 `port = COM14` 写死也行 |
| `[ERROR] 通信预检失败: 电机 N 不响应 (timeout 300ms)` | 电机没上电 / motor_id 配错 / CAN 总线断 |
| 编译报 `M_PI 未声明` | MSVC 不带这宏, 用本目录已经定义好的 `kPi` (cpp 文件顶部) |
| 链接报 `LNK2019 hightorque::*` | 没 link `hightorque_serial_debug`. 你应该 link `fafu_robot_sdk`, 它会传递依赖 |
| exe 启动报 `serial_cmake.dll 缺失` | 把 `serial_cmake.dll` 复制到 exe 同目录 (POST_BUILD 已经做了) |
