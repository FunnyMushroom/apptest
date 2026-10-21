#!/bin/sh
# ============================================================
# ota_push.sh — app923 一键编译 + OTA 推送
#
# 用法:
#   ./tools/ota_push.sh                    推送现成的 program_arm 到 192.168.0.106
#   ./tools/ota_push.sh --build            先 make STRIP=1 重新编译, 再推送
#   ./tools/ota_push.sh --build --host 192.168.0.100
#   ./tools/ota_push.sh 某个程序文件 [IP]   推指定文件(如桌面上的旧版做对照)
#
# 说明:
#   - 推送即 OTA 安装(mode 0), 装完【不会自动重启】, 需复位板子才生效
#   - 复位后新程序初始化完成会自动 ota_mark_ok, 不会触发回滚
#   - 完整日志在板上 /tmp/program_arm.log, 就绪标记 /tmp/program_arm.ready
# ============================================================

APP_DIR="$(cd "$(dirname "$0")/.." && pwd)"
UPLOAD="$APP_DIR/tools/ota_upload.py"
DEFAULT_FILE="$APP_DIR/program_arm"
DEFAULT_HOST="192.168.0.106"

usage() {
    sed -n '2,16p' "$0" | sed 's/^# \{0,1\}//'
    exit 0
}

BUILD=0
FILE=""
HOST=""

while [ $# -gt 0 ]; do
    case "$1" in
        --build)  BUILD=1 ;;
        --host)   shift; HOST="$1" ;;
        -h|--help) usage ;;
        *)
            if [ -z "$FILE" ]; then FILE="$1"; else HOST="$1"; fi
            ;;
    esac
    shift
done

[ -z "$FILE" ] && FILE="$DEFAULT_FILE"
[ -z "$HOST" ] && HOST="$DEFAULT_HOST"

cd "$APP_DIR" || exit 1

STEP=1
if [ "$BUILD" = "1" ]; then
    echo "==> [$STEP/3] 编译 (make STRIP=1) ..."
    make STRIP=1 || { echo "❌ 编译失败, 未推送"; exit 1; }
    STEP=2
else
    echo "==> [1/3] 跳过编译 (改过源码请加 --build)"
fi

if [ ! -f "$FILE" ]; then
    echo "❌ 找不到程序文件: $FILE"
    exit 1
fi

STEP=$((STEP+1))
echo "==> [$STEP/3] OTA 推送 $FILE -> $HOST ..."
python3 "$UPLOAD" --file "$FILE" --host "$HOST" --mode 0 || {
    echo "❌ 推送失败(检查网线/板子 IP 是否为 $HOST)"
    exit 1
}

echo "==> [3/3] 完成 ✔  记得复位板子, 新程序才生效。"
echo "    复位后看串口第一行: 程序版本: app923, 编译于 ..."
