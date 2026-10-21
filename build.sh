#!/usr/bin/env bash
set -euo pipefail

# 从脚本所在目录编译，避免依赖终端的当前目录。
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd -- "$SCRIPT_DIR"

# 使用当前 WSL 已安装的工具链，也允许通过环境变量覆盖。
CROSS="${CROSS:-$HOME/.local/opt/armv5-eabi--glibc--stable-2024.05-1/bin/arm-buildroot-linux-gnueabi-}"
OTA_HOST="${OTA_HOST:-192.168.0.106}"
OTA_PORT="${OTA_PORT:-8080}"

for tool in make python3 md5sum "${CROSS}gcc" "${CROSS}strip"; do
    if ! command -v "$tool" >/dev/null 2>&1; then
        echo "缺少构建工具：$tool" >&2
        exit 1
    fi
done

if [[ ! -f tools/ota_upload.py ]]; then
    echo "找不到 OTA 工具：$SCRIPT_DIR/tools/ota_upload.py" >&2
    exit 1
fi

echo '====================================='
echo "项目目录：$SCRIPT_DIR"
echo '开始编译 program_arm ...'
if ! make -B STRIP=1 CROSS="$CROSS" \
    CFLAGS='-O2 -g -Iinclude -D_GNU_SOURCE -mcpu=arm926ej-s -mfloat-abi=soft'; then
    echo '编译失败，已停止 OTA 上传。' >&2
    exit 1
fi

echo '====================================='
echo '编译成功，文件 MD5 校验值：'
md5sum program_arm

echo '====================================='
echo "正在通过 OTA 上传到 ${OTA_HOST}:${OTA_PORT} ..."
if ! python3 tools/ota_upload.py \
    --file ./program_arm \
    --host "$OTA_HOST" \
    --port "$OTA_PORT" \
    --mode 0; then
    echo 'OTA 上传失败，请检查上方错误及开发板网络。' >&2
    exit 1
fi

echo '====================================='
echo 'OTA 上传成功！请复位开发板使新程序生效。'
echo '====================================='
