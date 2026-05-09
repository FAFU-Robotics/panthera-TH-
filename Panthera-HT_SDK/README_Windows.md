# Panthera-HT Python SDK — Windows 11 源码编译指南

本文档提供在 **原生 Windows 11** 上从源码编译 Panthera-HT Python SDK 的完整步骤。
所有命令均在 **PowerShell** 中执行。

---

## 0. 前置条件

| 工具 | 最低版本 | 说明 |
|------|---------|------|
| Visual Studio 2022 | 17.x | 安装时勾选 **"使用 C++ 的桌面开发"** 工作负载 |
| CMake | 3.12+ | VS 自带，也可单独安装 |
| Git | 任意 | 用于克隆 vcpkg |
| Python | 3.9 – 3.12 | 建议用 conda 或官方安装包 |

---

## 1. 安装 vcpkg（包管理器）

如果你已有 vcpkg，跳到第 2 步。

```powershell
cd C:\
git clone https://github.com/microsoft/vcpkg.git
cd vcpkg
.\bootstrap-vcpkg.bat
```

设置环境变量（当前会话生效，建议也加到系统环境变量）：

```powershell
$env:VCPKG_ROOT = "C:\vcpkg"
$env:PATH += ";C:\vcpkg"
```

---

## 2. 用 vcpkg 安装 C++ 依赖

```powershell
vcpkg install yaml-cpp:x64-windows glib:x64-windows
```

> `yaml-cpp` 和 `glib` 是仅有的两个外部依赖。
> LCM 和 serial_cmake 已作为子目录包含在仓库里，无需额外安装。

---

## 3. 创建 Python 环境 & 安装 Python 依赖

```powershell
# conda 方式（推荐）
conda create -n panthera python=3.10 -y
conda activate panthera

# 安装编译期依赖
pip install pybind11

# 安装运行期依赖（高层库 Panthera_lib 需要）
cd F:\1LWP\Python\Panthera_Workspace\Panthera-HT_SDK\panthera_python
pip install -r requirements.txt
```

---

## 4. 编译 C++ 电机库（hightorque_motor）

```powershell
cd F:\1LWP\Python\Panthera_Workspace\Panthera-HT_SDK\panthera_cpp\motor_cpp

cmake -S . -B build -G "Visual Studio 17 2022" -A x64 `
    -DCMAKE_TOOLCHAIN_FILE="C:/vcpkg/scripts/buildsystems/vcpkg.cmake" `
    -DCMAKE_INSTALL_PREFIX="$env:USERPROFILE/panthera_install"

cmake --build build --config Release

cmake --install build --config Release
```

编译成功后，库文件安装到 `%USERPROFILE%\panthera_install`。

---

## 5. 编译 Python 绑定（_hightorque_robot.pyd）

```powershell
cd F:\1LWP\Python\Panthera_Workspace\Panthera-HT_SDK\panthera_python

cmake -S . -B build -G "Visual Studio 17 2022" -A x64 `
    -DCMAKE_TOOLCHAIN_FILE="C:/vcpkg/scripts/buildsystems/vcpkg.cmake" `
    -DCMAKE_PREFIX_PATH="$env:USERPROFILE/panthera_install"

cmake --build build --config Release
```

编译成功后会在 `panthera_python\hightorque_robot\` 目录下生成 `_hightorque_robot.pyd`。

---

## 6. 配置 DLL 搜索路径

Windows 需要能找到运行时 DLL。将以下路径加入 **系统 PATH**（或在每次使用前设置）：

```powershell
$env:PATH += ";$env:USERPROFILE\panthera_install\bin"
$env:PATH += ";C:\vcpkg\installed\x64-windows\bin"
```

> 建议将这两行写入系统环境变量，或放在 conda 环境的 activate 脚本里。

---

## 7. 验证安装

```powershell
python -c "import hightorque_robot; print('hightorque_robot OK')"
python -c "import pinocchio as pin; print('pin OK')"
python -c "import yaml; print('pyyaml OK')"
```

全部输出 OK 即安装成功。

---

## 8. 运行示例

```powershell
cd F:\1LWP\Python\Panthera_Workspace\Panthera-HT_SDK\panthera_python\scripts
python 0_robot_get_state.py
```

> **注意**：Windows 下串口名称是 `COM3`、`COM4` 等（而非 Linux 的 `/dev/ttyACM*`）。
> 你需要在 `robot_param/motor_param/*.yaml` 配置文件中把 `Serial_Type` 改为你的 COM 端口前缀，例如 `COM`。
> 可在"设备管理器 → 端口(COM 和 LPT)"中查看具体编号。

---

## 常见问题

### Q: CMake 报 `GLib2 not found`
确认 vcpkg 安装了 `glib:x64-windows`，并且 `-DCMAKE_TOOLCHAIN_FILE` 路径正确。

### Q: 运行时报找不到 DLL
把 `panthera_install\bin` 和 `vcpkg\installed\x64-windows\bin` 加入 PATH（见第 6 步）。

### Q: `find_package(hightorque_motor)` 失败
确认第 4 步 `cmake --install` 执行成功，并且第 5 步 `-DCMAKE_PREFIX_PATH` 指向了安装目录。

### Q: Python 找不到 `_hightorque_robot` 模块
确认第 5 步编译成功后 `panthera_python\hightorque_robot\` 目录下有 `_hightorque_robot.pyd` 文件。
如果 `.pyd` 生成在 `build\Release\` 下，手动拷贝到 `hightorque_robot\` 目录即可。

---

## 完整命令速查（一键复制）

```powershell
# ========== 一次性准备 ==========
# vcpkg
cd C:\
git clone https://github.com/microsoft/vcpkg.git
cd vcpkg
.\bootstrap-vcpkg.bat
$env:VCPKG_ROOT = "C:\vcpkg"
$env:PATH += ";C:\vcpkg"

# vcpkg 依赖
vcpkg install yaml-cpp:x64-windows glib:x64-windows

# Python 环境
conda create -n panthera python=3.10 -y
conda activate panthera
pip install pybind11

# ========== 编译 ==========
# C++ 电机库
cd F:\1LWP\Python\Panthera_Workspace\Panthera-HT_SDK\panthera_cpp\motor_cpp
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 -DCMAKE_TOOLCHAIN_FILE="C:/vcpkg/scripts/buildsystems/vcpkg.cmake" -DCMAKE_INSTALL_PREFIX="$env:USERPROFILE/panthera_install"
cmake --build build --config Release
cmake --install build --config Release

# Python 绑定
cd F:\1LWP\Python\Panthera_Workspace\Panthera-HT_SDK\panthera_python
pip install -r requirements.txt
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 -DCMAKE_TOOLCHAIN_FILE="C:/vcpkg/scripts/buildsystems/vcpkg.cmake" -DCMAKE_PREFIX_PATH="$env:USERPROFILE/panthera_install"
cmake --build build --config Release

# ========== 运行时 PATH ==========
$env:PATH += ";$env:USERPROFILE\panthera_install\bin;C:\vcpkg\installed\x64-windows\bin"

# ========== 验证 ==========
python -c "import hightorque_robot; print('OK')"
```
