#!/usr/bin/env bash
# =============================================================================
#  run_rt.sh — 给控制程序套上实时调度优先级
#
#  动机:
#    - 11_multi_joint_control 默认 250Hz (周期 4ms), 普通 SCHED_OTHER 调度下,
#      内核调度抖动可达 10-20ms (CPU 负载高时), 让 set_many 帧延迟巨大.
#    - SCHED_FIFO 优先级 50 + CPU affinity 固定到 1 个核, 抖动稳定在 < 1ms.
#
#  做的事:
#    1. 提高 RT 优先级上限 (ulimit -r 99), 不然普通用户没法 setpriority
#    2. taskset 把进程绑到指定 CPU (默认 0, 可改)
#    3. chrt -f 99 启动可执行文件
#
#  权限:
#    需要 CAP_SYS_NICE 或 root. 三种方案任选:
#      A. 单次 sudo:    sudo bash linux/run_rt.sh 11_multi_joint_control
#      B. 设 setcap (推荐, 一次性):
#            sudo setcap cap_sys_nice=eip build_linux/bin/11_multi_joint_control
#         然后不带 sudo 直接跑.
#      C. 改 /etc/security/limits.conf (老办法), 见 linux/README.md
#
#  用法:
#      bash linux/run_rt.sh 11_multi_joint_control        # 默认绑 CPU 0
#      bash linux/run_rt.sh 11_multi_joint_control --cpu 2 robot.cfg
#      bash linux/run_rt.sh 11_multi_joint_control --no-rt # 只 affinity, 不 RT
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_DIR="$( cd "$SCRIPT_DIR/.." && pwd )"
BIN_DIR="$PROJECT_DIR/build_linux/bin"

if [ $# -lt 1 ]; then
    echo "用法: bash linux/run_rt.sh <exe-name> [--cpu N] [--no-rt] [args...]"
    echo ""
    echo "可用程序:"
    if [ -d "$BIN_DIR" ]; then
        find "$BIN_DIR" -maxdepth 1 -type f -executable -printf "  %f\n" 2>/dev/null \
            | grep -v '\.so$' | sort
    else
        echo "  (build_linux/bin/ 不存在, 先跑 bash linux/build.sh)"
    fi
    exit 1
fi

EXE="$1"; shift
CPU=0
USE_RT=1
EXTRA_ARGS=()

while [ $# -gt 0 ]; do
    case "$1" in
        --cpu)    shift; CPU="$1" ;;
        --no-rt)  USE_RT=0 ;;
        --)       shift; EXTRA_ARGS+=("$@"); break ;;
        *)        EXTRA_ARGS+=("$1") ;;
    esac
    shift
done

EXE_PATH="$BIN_DIR/$EXE"
if [ ! -x "$EXE_PATH" ]; then
    echo "[run_rt] 找不到可执行文件: $EXE_PATH" >&2
    echo "         先跑 bash linux/build.sh 编译" >&2
    exit 1
fi

# 工具检查
for tool in taskset chrt; do
    if ! command -v "$tool" >/dev/null 2>&1; then
        echo "[run_rt] 错误: $tool 没装 (sudo apt install util-linux schedtool)" >&2
        exit 1
    fi
done

# 检查 RT 限制
RT_PRIO_LIMIT=$(ulimit -r 2>/dev/null || echo 0)
if [ "$USE_RT" -eq 1 ] && [ "$RT_PRIO_LIMIT" -lt 50 ] && [ "$(id -u)" -ne 0 ]; then
    echo "[run_rt] 警告: ulimit -r = $RT_PRIO_LIMIT, RT 优先级受限"
    echo "          解决方案:"
    echo "            A. 一次性给可执行文件加 cap_sys_nice:"
    echo "               sudo setcap cap_sys_nice=eip $EXE_PATH"
    echo "            B. 改 /etc/security/limits.conf 加: $USER hard rtprio 99"
    echo "            C. 加 sudo: sudo bash linux/run_rt.sh ..."
    echo ""
    echo "          这次尝试继续执行 (可能会被降级为 SCHED_OTHER):"
fi

echo "============================================================"
echo "  Panthera-HT 实时运行"
echo "  程序:        $EXE_PATH"
echo "  CPU 亲和:    $CPU"
echo "  调度策略:    $([ "$USE_RT" -eq 1 ] && echo 'SCHED_FIFO prio=80' || echo '默认')"
echo "  额外参数:    ${EXTRA_ARGS[*]:-(无)}"
echo "============================================================"

cd "$BIN_DIR"

# 用 -r 限制 80 (不是 99, 留几档给内核 RT 线程, 避免 RCU 饥饿)
if [ "$USE_RT" -eq 1 ]; then
    exec taskset -c "$CPU" chrt -f 80 "$EXE_PATH" "${EXTRA_ARGS[@]}"
else
    exec taskset -c "$CPU" "$EXE_PATH" "${EXTRA_ARGS[@]}"
fi
