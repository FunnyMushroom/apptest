#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
在线升级上传工具(上位机侧) —— 升级单个可执行文件 + 失败回滚

用法:
    python3 ota_upload.py --file ./program_arm --host 192.168.1.10 --mode 0

参数:
    --file PATH   新可执行文件(ELF)(mode 0/1 必填)
    --host IP     设备 IP(默认 127.0.0.1)
    --port PORT   命令端口(默认 8080)
    --mode 0|1|2  0=安装(默认) 1=只校验 2=立即回滚(无需 --file)

流程:
    1. 发 0x55 命令(大小+CRC32+模式)到命令端口
    2. 连文件端口 9090 上传新程序(裸字节)
    3. 收 0x56 应答并打印状态
"""
import argparse
import socket
import struct
import sys
import time

OTA_FILE_PORT = 9090
STATUS_NAMES = {
    0x00: "OK",
    0x01: "listen",
    0x02: "recv",
    0x03: "size",
    0x04: "crc",
    0x05: "toobig",
    0x06: "elf",
    0x07: "write",
    0x08: "backup",
    0x09: "nobackup",
    0x0A: "state",
    0x0B: "mode",
}


def crc32(data):
    table = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = (0xEDB88320 ^ (c >> 1)) if (c & 1) else (c >> 1)
        table.append(c & 0xFFFFFFFF)
    crc = 0xFFFFFFFF
    for b in data:
        crc = table[(crc ^ b) & 0xFF] ^ (crc >> 8)
    return crc ^ 0xFFFFFFFF


def frame(opcode, data):
    total = (b"\xab\xcd"
             + struct.pack(">I", len(data))
             + bytes([opcode])
             + b"\x00\x00"
             + bytes(data))
    cs = sum(b for i, b in enumerate(total) if i not in (7, 8)) & 0xFFFF
    return total[:7] + struct.pack(">H", cs) + bytes(data)


def main():
    ap = argparse.ArgumentParser(description="OTA single-binary uploader")
    ap.add_argument("--file", help="new ELF executable (mode 0/1)")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--mode", type=int, default=0, choices=[0, 1, 2],
                    help="0=install 1=verify 2=rollback (default 0)")
    args = ap.parse_args()

    if args.mode == 2:
        blob = b""
    else:
        if not args.file:
            raise SystemExit("--file required for mode 0/1")
        with open(args.file, "rb") as f:
            blob = f.read()
        if len(blob) < 4 or blob[:4] != b"\x7fELF":
            raise SystemExit("not an ELF file: %s" % args.file)
        print("file: %s (%d bytes), crc32=0x%08X" %
              (args.file, len(blob), crc32(blob)))

    # 1) 发 0x55 命令: [size(4)][crc32(4)][mode(1)]
    cmd = frame(0x55, struct.pack(">I", len(blob))
                + struct.pack(">I", crc32(blob) if blob else 0)
                + bytes([args.mode]))
    try:
        ctl = socket.create_connection((args.host, args.port), timeout=5)
    except OSError as e:
        raise SystemExit("cannot connect ctl %s:%d: %s" % (args.host, args.port, e))
    ctl.settimeout(30)
    ctl.sendall(cmd)

    # 2) 上传文件(mode 2 回滚不需要文件)
    if blob:
        # 板子收到命令后要走一整套: TCP 线程解析 -> 投递线程池 -> 某个 worker
        # 执行 Task_OTA_Upgrade -> socket/bind/listen, 需要一点时间。
        # 这里改成重试连接(最多约 10 秒), 而不是固定 sleep 一下就撞一次 9090。
        f = None
        last_err = None
        for _ in range(20):
            time.sleep(0.5)
            try:
                f = socket.create_connection((args.host, OTA_FILE_PORT), timeout=10)
                break
            except OSError as e:
                last_err = e
        if f is None:
            raise SystemExit("cannot connect file port %d after retries: %s"
                             % (OTA_FILE_PORT, last_err))
        f.sendall(blob)
        f.close()
        print("binary sent (%d bytes)" % len(blob))

    # 3) 收应答
    try:
        r = ctl.recv(64)
    except socket.timeout:
        r = b""
    ctl.close()
    if not r:
        raise SystemExit("no OTA reply received")
    print("OTA reply: %s" % r.hex())
    if len(r) >= 10 and r[0] == 0xAB and r[1] == 0xCD:
        status = r[9]
        print("status = %s(0x%02X)" % (STATUS_NAMES.get(status, "?"), status))
        sys.exit(0 if status == 0 else 1)
    raise SystemExit("bad reply frame")


if __name__ == "__main__":
    main()
