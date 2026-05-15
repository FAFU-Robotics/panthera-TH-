#!/usr/bin/env bash
# =============================================================================
#  build.sh — Ubuntu 一键 cmake 编译
#
#  - 在项目根目录的 build_linux/ 里构建 (与 Windows 的 build/ 隔离)
#  - 默认 Release + 并行编译, 用满 CPU
#  - 自动复制 robot.cfg 到产出目录 (CMakeLists 已经做了, 这里只是保险)
#  - 把 serial_cmake.so 加入 RPATH (cmake 默认行为, 这里只确认)
#
#  用法:
#      bash linux/build.sh                # Release, 并行, 增量
#      bash linux/build.sh --clean        # 删 build_linux/ 重头编
#      bash linux/build.sh --debug        # Debug 构建 (含 -O0 -g)
#      bash linux/build.sh --jobs 4       # 限制并行数
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_DIR="$( cd "$SCRIPT_DIR/.." && pwd )"
BUILD_DIR="$PROJECT_DIR/build_linux"

BUILD_TYPE=Release
DO_CLEAN=0
JOBS=$(nproc 2>/dev/null || echo 4)

while [ $# -gt 0 ]; do
    case "$1" in
        --debug)   BUILD_TYPE=Debug ;;
        --release) BUILD_TYPE=Release ;;
        --clean)   DO_CLEAN=1 ;;
        --jobs|-j) shift; JOBS="$1" ;;
        -h|--help)
            sed -n '2,15p' "$0"
            exit 0
            ;;
        *)
            echo "[build] 未知参数: $1" >&2
            exit 1
            ;;
    esac
    shift
done

cd "$PROJECT_DIR"

# ---- 工具链检查 ----
if ! command -v cmake >/dev/null 2>&1; then
    echo "[build] 错误: cmake 没装. 跑 bash linux/install_deps.sh 装上." >&2
    exit 1
fi
if ! command -v g++ >/dev/null 2>&1; then
    echo "[build] 错误: g++ 没装. 跑 bash linux/install_deps.sh 装上." >&2
    exit 1
fi

# ---- serial_cmake 路径检查 ----
SERIAL_DIR="$PROJECT_DIR/../motor_cpp/third_part/serial_cmake"
if [ ! -f "$SERIAL_DIR/CMakeLists.txt" ]; then
    echo "[build] 错误: 找不到 serial_cmake 第三方库" >&2
    echo "         期望路径: $SERIAL_DIR" >&2
    echo "         请确认目录结构: Panthera-HT_SDK/panthera_cpp/{motor_cpp,motor_example_debug}/" >&2
    exit 1
fi

echo "============================================================"
echo "  Panthera-HT 调试板 SDK — Linux 构建"
echo "  项目目录:    $PROJECT_DIR"
echo "  构建目录:    $BUILD_DIR"
echo "  构建类型:    $BUILD_TYPE"
echo "  并行 jobs:   $JOBS"
echo "  cmake:       $(cmake --version | head -n1)"
echo "  g++:         $(g++ --version | head -n1)"
echo "============================================================"

# ---- clean ----
if [ "$DO_CLEAN" -eq 1 ]; then
    echo "[build] --clean: 删除 $BUILD_DIR"
    rm -rf "$BUILD_DIR"
fi

# ---- configure ----
echo ""
echo "--- cmake configure ---"
cmake -S "$PROJECT_DIR" -B "$BUILD_DIR" \
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# 软链 compile_commands.json 到项目根, 方便 clangd / IDE 自动找
if [ -f "$BUILD_DIR/compile_commands.json" ]; then
    ln -sf "$BUILD_DIR/compile_commands.json" "$PROJECT_DIR/compile_commands.json"
fi

# ---- build ----
echo ""
echo "--- cmake build ---"
cmake --build "$BUILD_DIR" --parallel "$JOBS"

# ---- 列出产出 ----
echo ""
echo "============================================================"
echo "  构建完成. 产出在 $BUILD_DIR/bin/"
echo "============================================================"
ls -lh "$BUILD_DIR/bin/" | grep -v '^d' | grep -v '^total' || true

echo ""
echo "  跑示例:"
echo "    cd $BUILD_DIR/bin"
echo "    ./11_multi_joint_control                  # 主程序: 交互式多关节"
echo "    ./01_motor_get_status                     # 循环读 7 个电机状态"
echo "    ./13_test_many                            # 协议性能基准"
echo "    ./14_set_limits                           # 限位标定"
echo ""
echo "  或者用实时优先级跑 (250Hz 控制时减抖):"
echo "    bash linux/run_rt.sh 11_multi_joint_control"
