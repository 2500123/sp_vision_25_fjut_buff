#!/bin/bash

# 监控脚本：监督主程序运行，如果崩溃则自动重启

set -e

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXECUTABLE="${PROJECT_DIR}/build/standard_mpc"
LOG_FILE="${PROJECT_DIR}/logs/watchdog.log"
PID_FILE="${PROJECT_DIR}/watchdog.pid"

# 创建日志目录
mkdir -p "${PROJECT_DIR}/logs"

# 记录日志函数
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" | tee -a "${LOG_FILE}"
}

# 清理函数
cleanup() {
    log "Watchdog 进程收到退出信号，清理资源..."
    if [ ! -z "$PROGRAM_PID" ] && kill -0 "$PROGRAM_PID" 2>/dev/null; then
        log "杀死主程序（PID: $PROGRAM_PID）"
        kill "$PROGRAM_PID" 2>/dev/null || true
    fi
    rm -f "${PID_FILE}"
    log "Watchdog 已退出"
    exit 0
}

# 设置信号处理
trap cleanup SIGTERM SIGINT

log "===== Watchdog 启动 ====="
log "可执行文件: $EXECUTABLE"

# 检查可执行文件是否存在
if [ ! -f "$EXECUTABLE" ]; then
    log "[错误] 可执行文件不存在: $EXECUTABLE"
    exit 1
fi

# 保存 watchdog 的 PID
echo $$ > "${PID_FILE}"

# 主监控循环
RESTART_COUNT=0
while true; do
    log "启动主程序 (重启次数: $RESTART_COUNT)..."
    
    # 启动主程序
    if "$EXECUTABLE" &
    then
        PROGRAM_PID=$!
        log "主程序已启动 (PID: $PROGRAM_PID)"
        
        # 等待主程序退出
        wait $PROGRAM_PID
        EXIT_CODE=$?
        
        log "[警告] 主程序退出 (PID: $PROGRAM_PID, 退出码: $EXIT_CODE)"
        RESTART_COUNT=$((RESTART_COUNT + 1))
        
        # 如果程序异常退出，等待 2 秒后重启
        if [ $EXIT_CODE -ne 0 ]; then
            log "程序异常退出，2 秒后重启..."
            sleep 2
        else
            log "程序正常退出，2 秒后重启..."
            sleep 2
        fi
    else
        log "[错误] 无法启动主程序"
        sleep 5
        RESTART_COUNT=$((RESTART_COUNT + 1))
    fi
done
