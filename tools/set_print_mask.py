#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
打印开关控制工具(上位机侧) —— 单独控制每个(组)文件的打印开关

命令: 0x66, 数据 2 字节大端 = 16 位掩码
      bit = 1 -> 打开该模块打印;  bit = 0 -> 关闭
应答: 0x67, 数据 2 字节大端 = 已生效的掩码

用法示例:
    # 查看位号与文件的对应表
    python3 set_print_mask.py --list

    # 只关闭 spi.c 和 task.c 的打印(其余保持打开)
    python3 set_print_mask.py --host 192.168.1.10 --off spi,task

    # 只打开 i2c 和 fpga, 其余全部关闭
    python3 set_print_mask.py --host 192.168.1.10 --only i2c,fpga

    # 全部打开 / 全部关闭
    python3 set_print_mask.py --host 192.168.1.10 --all-on
    python3 set_print_mask.py --host 192.168.1.10 --all-off

    # 直接指定 16 位掩码
    python3 set_print_mask.py --host 192.168.1.10 --mask 0xFFFB
"""
import argparse
import socket
import struct
import sys

# 位号 -> 名称(必须与 include/log.h 里的 LOG_MOD_xxx 保持一致)
BITS = [
    (0,  "main",    "main.c, threadpool.c"),
    (1,  "i2c",     "i2c.c"),
    (2,  "spi",     "spi.c"),
    (3,  "gpio",    "gpio.c, gpio_app.c"),
    (4,  "usart",   "usart.c"),
    (5,  "rs485",   "uart485.c, 485_cpld.c"),
    (6,  "adc121",  "adc121s.c, realtime.c"),
    (7,  "ads1115", "ads1115.c"),
    (8,  "ads8691", "ads8691.c"),
    (9,  "dac8562", "dac8562.c"),
    (10, "fpga",    "fpga_i2c.c"),
    (11, "tcp",     "tcpserver.c"),
    (12, "net",     "net_config.c, eth.c"),
    (13, "task",    "task.c"),
    (14, "proto",   "protocol.c"),
    (15, "sys",     "ota.c, sd.c"),
]
NAME_TO_BIT = {name: bit for bit, name, _ in BITS}
ALL_MASK = 0xFFFF


def frame(opcode, data):
    """按协议组帧: AB CD | len(4,大端) | opcode | checksum(2,大端) | data"""
    total = (b"\xab\xcd"
             + struct.pack(">I", len(data))
             + bytes([opcode])
             + b"\x00\x00"
             + bytes(data))
    cs = sum(b for i, b in enumerate(total) if i not in (7, 8)) & 0xFFFF
    return total[:7] + struct.pack(">H", cs) + bytes(data)


def show_bits():
    print("位号  名称      对应文件                    当前默认")
    for bit, name, files in BITS:
        print("  %-3d %-9s %-26s 打开" % (bit, name, files))
    print()
    print("说明: 掩码 bit=1 打开, bit=0 关闭; 例 0xFFFB = 关闭 bit2(spi)")


def parse_names(text):
    bits = 0
    for name in text.split(","):
        name = name.strip().lower()
        if not name:
            continue
        if name not in NAME_TO_BIT:
            raise SystemExit("未知模块名: %s\n可用名称见 --list" % name)
        bits |= 1 << NAME_TO_BIT[name]
    return bits


def main():
    ap = argparse.ArgumentParser(
        description="设置设备端分模块打印开关(命令 0x66)")
    ap.add_argument("--host", default="127.0.0.1", help="设备 IP(默认 127.0.0.1)")
    ap.add_argument("--port", type=int, default=8080, help="命令端口(默认 8080)")
    ap.add_argument("--mask", help="直接指定 16 位掩码, 支持 0x 前缀")
    ap.add_argument("--on", help="打开的模块名, 逗号分隔(其余保持不变)")
    ap.add_argument("--off", help="关闭的模块名, 逗号分隔(其余保持不变)")
    ap.add_argument("--only", help="只打开这些模块, 其余全部关闭")
    ap.add_argument("--all-on", action="store_true", help="全部打开(0xFFFF)")
    ap.add_argument("--all-off", action="store_true", help="全部关闭(0x0000)")
    ap.add_argument("--list", action="store_true", help="只打印位号对照表, 不连接设备")
    args = ap.parse_args()

    if args.list:
        show_bits()
        return

    # ---- 计算掩码 ----
    if args.mask is not None:
        mask = int(args.mask, 0) & ALL_MASK
    elif args.only is not None:
        mask = parse_names(args.only)
    elif args.all_off:
        mask = 0x0000
    elif args.all_on:
        mask = ALL_MASK
    elif args.on is not None or args.off is not None:
        mask = ALL_MASK                      # 以"全开"为基准增删
        if args.on:
            mask |= parse_names(args.on)
        if args.off:
            mask &= ~parse_names(args.off)
        mask &= ALL_MASK
    else:
        show_bits()
        print("\n请至少指定一个操作: --on / --off / --only / --mask / --all-on / --all-off")
        return

    # ---- 打印将要下发的清单 ----
    print("将要下发的打印开关掩码: 0x%04X" % mask)
    for bit, name, files in BITS:
        print("  bit%-2d %-9s %-26s %s" %
              (bit, name, files, "打开" if (mask >> bit) & 1 else "关闭"))

    # ---- 下发 0x66 ----
    cmd = frame(0x66, struct.pack(">H", mask))
    try:
        sock = socket.create_connection((args.host, args.port), timeout=5)
    except OSError as e:
        raise SystemExit("无法连接 %s:%d: %s" % (args.host, args.port, e))

    sock.settimeout(5)
    sock.sendall(cmd)
    try:
        reply = sock.recv(64)
    except socket.timeout:
        reply = b""
    sock.close()

    if not reply:
        raise SystemExit("未收到应答(0x67)")
    print("应答: %s" % reply.hex())
    # 回包布局: [0..1]=ABCD [2..5]=len [6]=opcode [7..8]=checksum [9..]=数据
    if len(reply) >= 11 and reply[0] == 0xAB and reply[1] == 0xCD:
        opcode = reply[6]
        applied = (reply[9] << 8) | reply[10]
        print("回包 opcode = 0x%02X, 已生效掩码 = 0x%04X" % (opcode, applied))
        if opcode != 0x67:
            print("⚠️ 回包操作码不是 0x67, 请检查固件是否已支持 0x66")
        if applied != mask:
            print("⚠️ 生效掩码与下发不一致: 下发 0x%04X, 生效 0x%04X" % (mask, applied))
            sys.exit(1)
    else:
        raise SystemExit("回包格式异常: %s" % reply.hex())


if __name__ == "__main__":
    main()
