#!/usr/bin/env bash
# =============================================================================
#  install_deps.sh — Ubuntu 一键装依赖
#
#  做的事:
#    1. apt 装编译工具链 + cmake + git (build-essential 包含 gcc/g++/make/libc-dev)
#    2. apt 装 udev / pkg-config (serial_cmake 在 Linux 上链 librt + libpthread, 都在 libc6-dev 里; 不需要 libudev)
#    3. 把当前用户加入 dialout 组 (访问 /dev/ttyUSB* 必需)
#    4. 提示重新登录使组生效
#
#  支持系统: Ubuntu 18.04 / 20.04 / 22.04 / 24.04, Debian 11+
#
#  用法:
#      bash linux/install_deps.sh
#      bash linux/install_deps.sh --skip-group   # 不动用户组 (CI / Docker 里用)
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_DIR="$( cd "$SCRIPT_DIR/.." && pwd )"

SKIP_GROUP=0
for arg in "$@"; do
    case "$arg" in
        --skip-group) SKIP_GROUP=1 ;;
        -h|--help)
            sed -n '2,20p' "$0"
            exit 0
            ;;
        *)
            echo "[install_deps] 未知参数: $arg" >&2
            exit 1
            ;;
    esac
done

echo "============================================================"
echo "  Panthera-HT 调试板 SDK — Ubuntu 依赖安装"
echo "  项目目录: $PROJECT_DIR"
echo "============================================================"

# ---- 1. 检测系统 ----
if [ ! -f /etc/os-release ]; then
    echo "[install_deps] 无法检测系统 (找不到 /etc/os-release)" >&2
    exit 1
fi
. /etc/os-release
echo "[install_deps] 检测到系统: $PRETTY_NAME"

case "${ID:-}" in
    ubuntu|debian|linuxmint|pop) ;;
    *)
        echo "[install_deps] 警告: 当前系统是 '$ID', 仅在 Ubuntu/Debian 系测试过."
        echo "                如果你的发行版用 dnf/pacman/zypper, 请手动装等价包并跳过本步."
        read -r -p "                继续 (y/N)? " ans
        [[ "$ans" =~ ^[Yy]$ ]] || exit 1
        ;;
esac

# ---- 2. apt 装包 ----
echo ""
echo "--- 1/3 apt 装编译依赖 ---"
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    pkg-config \
    git \
    ca-certificates

# build-essential 已经包含 gcc/g++/make/libc6-dev/libstdc++-dev
# serial_cmake 链的 rt/pthread 在 libc6-dev 里, 不需要额外装

# ---- 3. 校验 ----
echo ""
echo "--- 2/3 工具链版本校验 ---"
echo "  gcc:    $(gcc --version | head -n1)"
echo "  g++:    $(g++ --version | head -n1)"
echo "  cmake:  $(cmake --version | head -n1)"

GCC_MAJOR=$(gcc -dumpversion | cut -d. -f1)
if [ "$GCC_MAJOR" -lt 7 ]; then
    echo "[install_deps] 错误: gcc 主版本 $GCC_MAJOR < 7, 不支持 C++17 (需要 gcc-7+)" >&2
    echo "                Ubuntu 18.04 可用: sudo apt install gcc-8 g++-8 && export CC=gcc-8 CXX=g++-8" >&2
    exit 1
fi

CMAKE_VER=$(cmake --version | head -n1 | grep -oE '[0-9]+\.[0-9]+' | head -n1)
CMAKE_MAJOR=${CMAKE_VER%.*}
CMAKE_MINOR=${CMAKE_VER#*.}
if [ "$CMAKE_MAJOR" -lt 3 ] || { [ "$CMAKE_MAJOR" -eq 3 ] && [ "$CMAKE_MINOR" -lt 12 ]; }; then
    echo "[install_deps] 错误: cmake $CMAKE_VER < 3.12, 不满足 CMakeLists.txt 要求" >&2
    exit 1
fi

# ---- 4. dialout 组 ----
echo ""
echo "--- 3/3 用户组 (访问 /dev/ttyUSB* 必需) ---"

if [ "$SKIP_GROUP" -eq 1 ]; then
    echo "[install_deps] --skip-group 已指定, 跳过用户组处理"
else
    if id -nG "$USER" | grep -qw dialout; then
        echo "[install_deps] 用户 $USER 已在 dialout 组"
    else
        echo "[install_deps] 把 $USER 加入 dialout 组..."
        sudo usermod -aG dialout "$USER"
        echo ""
        echo "  *** 重要: 用户组修改需要 *重新登录* 才生效 ***"
        echo "  退出当前 shell / 桌面会话后重新登录, 或者跑:"
        echo "      newgrp dialout      # 仅当前 shell 临时生效"
    fi
fi

echo ""
echo "============================================================"
echo "  依赖安装完成. 下一步:"
echo "    1. (可选) 安装 udev 规则:    bash linux/setup_udev.sh"
echo "    2. 编译:                       bash linux/build.sh"
echo "    3. 跑示例:                     ./build_linux/bin/11_multi_joint_control"
echo "============================================================"
