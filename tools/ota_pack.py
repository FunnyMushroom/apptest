#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OTA 固件打包与远程上传工具(上位机侧)

用法示例:
    python3 ota_pack.py --dtb board.dtb --kernel zImage --rootfs rootfs.jffs2 \
            --out pkg.bin --host 192.168.1.10 --port 8080 --mode 0

参数:
    --uboot FILE     (可选) uboot 镜像
    --dtb FILE       (可选) 设备树镜像
    --kernel FILE    (可选) 内核镜像
    --rootfs FILE    (可选) rootfs 镜像(JFFS2)
    --out FILE       生成的固件包路径(必填)
    --host IP        设备 IP(默认 127.0.0.1)
    --port PORT      命令端口(默认 8080)
    --mode 0|1|2     0=apply 1=verify 2=stage(默认 0)

至少指定一个镜像。打包后自动通过 TCP 上传并打印设备应答。
"""
import argparse
import socket
import struct
import sys
import time

OTA_MAGIC = b"CESHIOTA"
OTA_FILE_PORT = 9090
TYPES = {"uboot": 0x01, "dtb": 0x02, "kernel": 0x03, "rootfs": 0x04}
STATUS_NAMES = {
    0x00: "OK",
    0x01: "listen",
    0x02: "recv",
    0x03: "size",
    0x04: "header",
    0x05: "crc",
    0x06: "toobig",
    0x07: "write",
    0x08: "rootbusy",
}


def crc32(data):
    """标准 CRC-32/IEEE(与设备端 ota_crc32 一致)"""
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


def build_package(images):
    """images: list of (name, type, bytes) -> 固件包 bytes"""
    if not 1 <= len(images) <= 8:
        raise SystemExit("image count must be 1..8")

    header = bytearray(OTA_MAGIC)
    header += struct.pack(">I", 1)              # version = 1
    header += bytes([len(images)])              # image count
    header += b"\x00\x00\x00"                   # reserved

    entries = b""
    payloads = b""
    for _name, typ, data in images:
        entries += (bytes([typ])
                    + struct.pack(">I", len(data))
                    + struct.pack(">I", crc32(data))
                    + b"\x00" * 7)
        payloads += data
    return bytes(header) + entries + payloads


def frame(opcode, data):
    """构造设备协议帧(与设备端 checksum 算法一致)"""
    total = (b"\xab\xcd"
             + struct.pack(">I", len(data))
             + bytes([opcode])
             + b"\x00\x00"
             + bytes(data))
    cs = sum(b for i, b in enumerate(total) if i not in (7, 8)) & 0xFFFF
    return total[:7] + struct.pack(">H", cs) + bytes(data)


def main():
    ap = argparse.ArgumentParser(description="OTA package builder & uploader")
    for t in TYPES:
        ap.add_argument("--%s" % t, metavar="FILE", help="%s image file" % t)
    ap.add_argument("--out", required=True, help="output package path")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--mode", type=int, default=0, choices=[0, 1, 2],
                    help="0=apply 1=verify 2=stage (default 0)")
    args = ap.parse_args()

    images = []
    for name, typ in TYPES.items():
        path = getattr(args, name)
        if path:
            with open(path, "rb") as f:
                data = f.read()
            images.append((name, typ, data))
            print("add %-6s %-24s %8d bytes" % (name, path, len(data)))
    if not images:
        raise SystemExit("no image specified")

    pkg = build_package(images)
    with open(args.out, "wb") as f:
        f.write(pkg)
    print("package: %s (%d bytes, %d images)" % (args.out, len(pkg), len(images)))

    # 1) 发 0x23 命令(总大小 + 模式)
    cmd = frame(0x23, struct.pack(">I", len(pkg)) + bytes([args.mode, 0]))
    try:
        ctl = socket.create_connection((args.host, args.port), timeout=5)
    except OSError as e:
        raise SystemExit("cannot connect ctl %s:%d: %s" % (args.host, args.port, e))
    ctl.settimeout(15)
    ctl.sendall(cmd)

    # 2) 连文件端口 9090 发送固件包
    time.sleep(0.3)
    try:
        f = socket.create_connection((args.host, OTA_FILE_PORT), timeout=10)
    except OSError as e:
        raise SystemExit("cannot connect file port %d: %s" % (OTA_FILE_PORT, e))
    f.sendall(pkg)
    f.close()
    print("firmware sent (%d bytes)" % len(pkg))

    # 3) 收应答
    try:
        r = ctl.recv(64)
    except socket.timeout:
        r = b""
    ctl.close()
    if not r:
        raise SystemExit("no OTA reply received")
    print("OTA reply: %s" % r.hex())

    if len(r) >= 11 and r[0] == 0xAB and r[1] == 0xCD:
        status = r[9]
        img = r[10]
        print("status = %s(0x%02X), img_idx = %d" %
              (STATUS_NAMES.get(status, "?"), status, img))
        sys.exit(0 if status == 0 else 1)
    raise SystemExit("bad reply frame")


if __name__ == "__main__":
    main()
