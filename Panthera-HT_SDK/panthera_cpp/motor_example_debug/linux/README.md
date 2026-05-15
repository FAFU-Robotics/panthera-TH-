# Ubuntu 部署指南

这套文件是把 `motor_example_debug` 跑在 **物理 Ubuntu 机器** 上的全套配套。
代码本身已经是跨平台的（`hightorque_serial.cpp`、`11_multi_joint_control.cpp`
等都带 `#ifdef _WIN32 / #else POSIX` 双分支），所以这里**不重写 C++ 代码**，
只补 Ubuntu 上跑起来需要的：装依赖、USB 权限、构建、实时调度、配置示例。

## 支持的发行版

| 系统 | 内核版本 | g++ | cmake | 测试状态 |
|------|----------|-----|-------|---------|
| Ubuntu 22.04 LTS | 5.15+ | 11.x | 3.22 | 推荐 |
| Ubuntu 20.04 LTS | 5.4+ | 9.x | 3.16 | OK |
| Ubuntu 24.04 LTS | 6.5+ | 13.x | 3.28 | OK |
| Ubuntu 18.04 LTS | 4.15+ | 7.x | 3.10 | 需手动装 cmake 3.12+ |
| Debian 11 / 12 | — | — | — | 同等 |

最低要求：**C++17（g++ 7+ / clang 5+）+ cmake 3.12+**。

## 从 0 到 1 部署（5 步）

```bash
# 0) 进项目目录
cd Panthera-HT_SDK/panthera_cpp/motor_example_debug

# 1) 装依赖 + 加入 dialout 组
bash linux/install_deps.sh

# 2) (推荐) 装 udev 规则, 让 USB 串口固定到 /dev/panthera_debug_board, 任何用户可读写
bash linux/setup_udev.sh

# 3) 插上调试板 USB, 检查能不能看到
ls -l /dev/ttyUSB* /dev/panthera_debug_board 2>/dev/null
# 期望: 至少一个 /dev/ttyUSBx 出现, 装了 udev 规则的话还会有 panthera_debug_board

# 4) 编译
bash linux/build.sh

# 5) 跑示例
./build_linux/bin/01_motor_get_status        # 先用这个验证通信
./build_linux/bin/11_multi_joint_control     # 主程序: 交互式多关节控制
```

第一次跑前如果遇到 `Permission denied`：要么 **重新登录** 让 dialout 组生效，
要么 `newgrp dialout` 临时生效，要么用了 udev 规则的话直接走 `/dev/panthera_debug_board`
就不用关心组的事。

## 文件一览

```
linux/
├── README.md                            # 你正在看的文件
├── install_deps.sh                      # apt 装依赖 + 加入 dialout 组
├── setup_udev.sh                        # 装 / 卸载 / 检查 udev 规则
├── 99-panthera-debug-board.rules        # udev 规则源文件
├── build.sh                             # cmake 一键编译到 build_linux/
├── run_rt.sh                            # SCHED_FIFO + CPU affinity 启动 (减抖)
└── robot.linux.cfg                      # Linux 版示例配置
```

可执行文件统一产出到 `build_linux/bin/`，与 Windows 用的 `build/` 完全隔离，
你可以同一份代码在 WSL 和 Windows 上同时维护两套构建。

## 1. 安装依赖 `install_deps.sh`

```bash
bash linux/install_deps.sh
```

做的事：

1. 用 `apt` 装 `build-essential`、`cmake`、`pkg-config`、`git`
2. 校验 `gcc >= 7`、`cmake >= 3.12`（不满足直接报错退出）
3. 把当前用户加入 `dialout` 组（访问 `/dev/ttyUSB*` 必需）

⚠ 加完组**必须重新登录**，或者临时 `newgrp dialout` 才生效。
跑完 `id -nG` 看看输出里有没有 `dialout` 字样确认。

如果在 Docker 或 CI 里跑，不想动用户组：

```bash
bash linux/install_deps.sh --skip-group
```

## 2. USB 权限：udev 规则 `setup_udev.sh`

`/dev/ttyUSB0` 默认权限是 `crw-rw---- root dialout`，只有 `dialout` 组成员能读写，
而且每次插的口不一样路径还会变（`ttyUSB0` → `ttyUSB1`），把端口写死在 `robot.cfg`
里就麻烦。装一份 udev 规则就解决了：

```bash
# 安装 (会 sudo)
bash linux/setup_udev.sh

# 检查规则是否生效 (不动任何东西)
bash linux/setup_udev.sh --verify

# 卸载
bash linux/setup_udev.sh --uninstall
```

装完之后：

- 调试板插上后自动出现 `/dev/panthera_debug_board`（软链 → 真实的 `/dev/ttyUSBx`）
- 权限改成 `0666`（任何用户都能读写，不必加 `dialout` 组）
- 拔了重新插，路径都不变

`robot.cfg` 里就可以写：

```
port = /dev/panthera_debug_board
```

⚠ 已经默认覆盖了 ST/FTDI/CH340/CP210x/PL2303 五大常见 USB-Serial 桥芯片。如果你的
调试板 VID 不在表里，需要先查：

```bash
lsusb                                # 找到调试板那一行, ID 字段前 4 位是 VID
udevadm info -a -n /dev/ttyUSB0 | grep -m1 idVendor   # 备选方法
```

然后在 `99-panthera-debug-board.rules` 里照着已有行加一条 `ATTRS{idVendor}=="xxxx"`，
重新跑 `bash linux/setup_udev.sh`。

## 3. 编译 `build.sh`

```bash
bash linux/build.sh                       # Release + nproc 并行 (默认)
bash linux/build.sh --clean               # 删 build_linux/ 重新编
bash linux/build.sh --debug               # Debug 构建 (-O0 -g)
bash linux/build.sh --jobs 4              # 限制 4 个并行
```

产出全部在 `build_linux/bin/`：

```
build_linux/bin/
├── 01_motor_get_status      # 循环读 7 个电机状态 (验证通信用)
├── 09_set_zero              # 重置零点 (交互选关节)
├── 11_multi_joint_control   # ★ 主程序: 交互式多关节 250Hz + S 曲线
├── 12_demo_new_features     # 新能力演示
├── 13_test_many             # 一拖多 vs 单发 性能基准
├── 14_set_limits            # 软限位标定工具
├── libserial_cmake.so       # 串口库 (跟 exe 同目录, RPATH 已配)
└── robot.cfg                # 配置文件 (CMakeLists 自动复制)
```

`compile_commands.json` 会自动软链到项目根，clangd / VSCode / nvim 能直接用。

### 在 Ubuntu 18.04 上 cmake 太老

```bash
# 卸载 apt 版 cmake (太老), 装 Kitware 官方源
sudo apt remove --purge cmake
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null \
    | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null
echo "deb https://apt.kitware.com/ubuntu/ bionic main" \
    | sudo tee /etc/apt/sources.list.d/kitware.list
sudo apt update && sudo apt install -y cmake
```

## 4. 跑示例

```bash
cd build_linux/bin

# A) 最简单的: 循环打印 7 关节当前位置 (Ctrl+C 退出, 关节可手动转动)
./01_motor_get_status

# B) 主交互程序 (输入 "all 0 0 0 0 0 0 0" 让全部回零)
./11_multi_joint_control
# 同等支持 ESC / Ctrl+Q 紧急停止 (Linux 终端 ESC 永远直传; Ctrl+Q 不一定)

# C) 协议性能测试 (不动电机, 只发查询和 stop)
./13_test_many

# D) 软限位标定
./14_set_limits
```

## 5. (可选) 实时调度减抖 `run_rt.sh`

控制频率 100Hz 普通用户就能跑。**250Hz / 500Hz 推荐配 SCHED_FIFO**，否则被
其它 CPU 密集任务抢核会出 max_jitter 飙到 10ms+ 的尖刺。

```bash
# 一次性给可执行文件加 cap_sys_nice (推荐, 不用每次 sudo)
sudo setcap cap_sys_nice=eip build_linux/bin/11_multi_joint_control

# 跑
bash linux/run_rt.sh 11_multi_joint_control          # 默认绑 CPU 0
bash linux/run_rt.sh 11_multi_joint_control --cpu 2  # 绑到 CPU 2
bash linux/run_rt.sh 11_multi_joint_control --no-rt  # 只 affinity 不 RT (温和方案)

# 带参数:
bash linux/run_rt.sh 11_multi_joint_control --cpu 2 -- linux/robot.linux.cfg
```

进一步降抖的进阶手段（可选）：

| 措施 | 效果 | 代价 |
|------|------|------|
| `setcap cap_sys_nice=eip` | 不用 root 也能拿 RT 优先级 | 一次性, 升级二进制后要重新 setcap |
| `taskset -c N`（CPU 亲和） | 绑核, cache 利用率高 | 占用一个核 |
| `chrt -f 80`（SCHED_FIFO） | 抢占普通用户进程 | RT 调度内核 |
| `isolcpus=N` 内核 cmdline | 把 CPU N 完全交给你 | 改 GRUB, 重启 |
| PREEMPT_RT 内核 | 内核内部也变可抢占 | 装第三方内核 |

普通 Ubuntu + setcap + SCHED_FIFO + taskset 在 250Hz 已经够用，`max_jitter`
一般稳定在 < 1.5ms（程序自己会打印 stats，可以看到）。

## 6. 配置文件

`build.sh` 编完之后 CMakeLists 会自动复制项目根的 `robot.cfg` 到 `build_linux/bin/`，
所以可执行文件运行时**优先用同目录下的 `robot.cfg`**。

Linux 上想要不同配置：

```bash
# 方案 A: 覆盖根目录的 robot.cfg (Windows 用同一份)
cp linux/robot.linux.cfg robot.cfg

# 方案 B: 单独传配置, 不污染根目录
./build_linux/bin/11_multi_joint_control linux/robot.linux.cfg
```

`linux/robot.linux.cfg` 与项目根的 `robot.cfg` 字段完全相同，唯一区别是 `port`
的注释里推荐 Linux 路径（`/dev/panthera_debug_board` 等）。

## 7. 故障排查

### 7.1 `Permission denied: '/dev/ttyUSB0'`

**最常见**。三种解决方案，**任选一种**：

| 方案 | 命令 | 持久 |
|------|------|------|
| A. 装 udev 规则（推荐） | `bash linux/setup_udev.sh` | ✅ |
| B. 加入 dialout 组 | `sudo usermod -aG dialout $USER` + **重新登录** | ✅ |
| C. 临时改权限 | `sudo chmod 0666 /dev/ttyUSB0` | ❌ 重启失效 |

判断属于哪一种问题：

```bash
ls -l /dev/ttyUSB0
# crw-rw---- 1 root dialout ...   ← 标准 (需要 dialout 组)
# crw-rw-rw- 1 root dialout ...   ← udev 规则生效后 (任何用户都能读写)
```

### 7.2 插了 USB 但 `/dev/ttyUSB*` 不出现

```bash
# 1. 看内核日志, 确认 USB 设备被识别了
sudo dmesg | tail -n 20
# 期望看到类似:
#   usb 3-1: new full-speed USB device ...
#   cdc_acm 3-1:1.0: ttyACM0: USB ACM device
# 或:
#   ch341 3-1:1.0: ch341-uart converter detected
#   usb 3-1: ch341-uart converter now attached to ttyUSB0

# 2. 看 lsusb 有没有
lsusb

# 3. 如果是 CH340/CH341 没识别 (Ubuntu 18 老内核):
sudo modprobe ch341

# 4. 如果是 CP210x:
sudo modprobe cp210x
```

### 7.3 `Could not open serial port: ...`

```bash
# 看是不是被另一个进程占用了
sudo lsof /dev/ttyUSB0
# 或
sudo fuser -v /dev/ttyUSB0

# ModemManager 是常见元凶 (Network Manager 会自动 probe 串口看是不是 3G/4G modem):
sudo systemctl stop ModemManager
sudo systemctl disable ModemManager   # 永久禁用 (你应该不用 3G/4G 上网)
```

### 7.4 `cmake configure` 报 `未找到 serial_cmake`

```
fatal error: 未找到 serial_cmake: .../motor_cpp/third_part/serial_cmake
```

意味着目录结构不对。这套代码需要 `serial_cmake` 在 `../motor_cpp/third_part/`，
即完整的 SDK 布局：

```
Panthera-HT_SDK/
└── panthera_cpp/
    ├── motor_cpp/
    │   └── third_part/
    │       └── serial_cmake/        ← 必须存在
    └── motor_example_debug/         ← 你在这跑 cmake
```

如果你只 clone 了一部分子目录，去仓库根再 `git clone` 一遍完整结构。

### 7.5 编译报 `error: 'std::optional' ...`

g++ 版本太老。检查：

```bash
g++ --version
# 必须 >= 7. Ubuntu 18.04 默认 g++ 是 7.5, 刚好够用; 16.04 默认 5.x, 要升级:
sudo apt install g++-8
export CC=gcc-8 CXX=g++-8
bash linux/build.sh --clean
```

### 7.6 跑起来一切正常但 `max_jitter` 很大 / 控制不顺

按这个顺序试：

1. 把 `control_rate_hz` 从 250 降到 100，看是不是好了（确认是抖动问题不是协议问题）
2. 用 `linux/run_rt.sh` 加 SCHED_FIFO + CPU affinity
3. `sudo systemctl stop ModemManager`（前面提过）
4. 关掉浏览器、IDE、Docker daemon 等 CPU hog
5. CPU governor 调成 performance：
   ```bash
   sudo apt install linux-tools-common linux-tools-generic
   sudo cpupower frequency-set -g performance
   ```
6. 内核 cmdline 加 `isolcpus=3 nohz_full=3 rcu_nocbs=3`（编辑 `/etc/default/grub` →
   `sudo update-grub` → 重启），然后 `--cpu 3` 让控制环独占 CPU 3

绝大多数情况下做到第 2 步就够了。

### 7.7 WSL2 上能跑吗

可以，但要注意：

- 默认 WSL2 是 `usbipd-win` 直通 USB，跑前要在 Windows 侧执行 `usbipd attach`
- WSL2 没有 RT 内核，`SCHED_FIFO` 退化为普通调度，`max_jitter` 比物理机大
- 250Hz 控制环建议跑物理 Ubuntu，WSL2 主要用来开发调试

你选的是物理 Ubuntu 机器，可以忽略这一段。

## 8. 与现有代码的关系

不改任何 C++ 文件，**不破坏 Windows 构建**。所有 Linux 特定的脚本都在
`linux/` 子目录里独立维护。

| 代码层 | 改动 |
|--------|------|
| `src/*.cpp` | ❌ 不动（早就有 `#ifdef _WIN32 / #else` 双分支） |
| `include/*.hpp` | ❌ 不动 |
| `CMakeLists.txt` | ❌ 不动（已经 `if(MSVC) else()` 分支） |
| `robot.cfg` | ❌ 不动（注释里已经提了 `/dev/ttyUSB0`） |
| `linux/*` | ✅ 全新 |

升级 SDK 时 `linux/` 里的东西不会被覆盖。

## 9. 卸载

```bash
# 1. 删 build 产出
rm -rf build_linux/

# 2. 卸载 udev 规则
bash linux/setup_udev.sh --uninstall

# 3. 移除 dialout 组成员 (可选, 一般不需要)
sudo gpasswd -d $USER dialout
```
