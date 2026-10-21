#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
app923 调试用: 发一条协议命令并打印应答(不依赖串口日志)。

协议帧: AB CD | data_len(4,大端) | opcode(1) | checksum(2,大端) | data
checksum = 除校验位外所有字节累加 & 0xFFFF

用法:
    python3 dbg_cmd.py <opcode(十六进制)> [data 十六进制字节...]

例:
    python3 dbg_cmd.py 0x0B 0000     # 自检: 读 FPGA 0x04
    python3 dbg_cmd.py 0x07 00       # 读高压(U, ADS1115 原始码)
    python3 dbg_cmd.py 0x1F 00       # 读电流(ADS1115 原始码)
    python3 dbg_cmd.py 0xFD 05 01    # 写 0x05=0x01, 回读 0x04/0x05 → 0xFE
    python3 dbg_cmd.py 0xFD FF 02    # 特殊值: LDAC(PD13) 脉冲

环境变量: DBG_HOST(默认 192.168.0.100), DBG_PORT(默认 8080)
"""
import os
import socket
import struct
import sys
import time

HOST = os.environ.get("DBG_HOST", "192.168.0.106")
PORT = int(os.environ.get("DBG_PORT", "8080"))


def frame(opcode, data=b""):
    total = (b"\xab\xcd" + struct.pack(">I", len(data)) + bytes([opcode])
             + b"\x00\x00" + bytes(data))
    cs = sum(b for i, b in enumerate(total) if i not in (7, 8)) & 0xFFFF
    return total[:7] + struct.pack(">H", cs) + bytes(data)


def recv_reply(sock, timeout=8.0):
    sock.settimeout(timeout)
    buf = b""
    end = time.time() + timeout
    while time.time() < end:
        try:
            chunk = sock.recv(4096)
        except socket.timeout:
            break
        if not chunk:
            break
        buf += chunk
        if len(buf) >= 9:
            ln = struct.unpack(">I", buf[2:6])[0]
            if len(buf) >= 9 + ln:
                return buf[:9 + ln]
    return buf


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    op = int(sys.argv[1], 16)
    h = "".join(sys.argv[2:]).replace(" ", "").replace(",", "")
    data = bytes.fromhex(h) if h else b""

    req = frame(op, data)
    print("TX:", req.hex(" "))
    try:
        s = socket.create_connection((HOST, PORT), timeout=5)
    except OSError as e:
        print("连接 %s:%d 失败: %s" % (HOST, PORT, e))
        return 2
    s.sendall(req)
    r = recv_reply(s)
    s.close()

    if len(r) < 9:
        print("RX(不完整/无应答):", r.hex(" "))
        return 3
    ln = struct.unpack(">I", r[2:6])[0]
    print("RX:", r.hex(" "))
    print("  opcode=0x%02X len=%d data=%s" % (r[6], ln, r[9:9 + ln].hex(" ")))
    return 0


if __name__ == "__main__":
    sys.exit(main())
