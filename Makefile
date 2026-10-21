# ============================================================
#  NUC980 应用构建  (app1)  ——  ARM 专用
#
#  目标平台: NUC980 IoT (ARM926EJ-S, ARMv5TEJ)
#  工具链  : buildroot 编出的 Nuvoton 交叉编译器 (gcc 12.3.0)
#
#  用法:
#     make            编 program_arm —— 静态链接的 ARM 可执行文件, 给板子
#     make clean      清掉产物
#
#  常用变量:
#     make DEBUG=0                不带 -g, 产物小 ~170K
#     make STRIP=1                编完顺手 strip 掉符号表
#     make CROSS=/path/to/prefix- 换工具链前缀
# ============================================================

# ---- 工具链(交叉编译, 不是本机 gcc)--------------------------
CROSS ?= /home/hua123/nuc980/buildroot_2024/output/host/bin/arm-nuvoton-linux-gnueabi-
CC     = $(CROSS)gcc

# ---- 目录 / 源文件 -------------------------------------------
SRCDIR = src
INCDIR = include
SRCS   = $(wildcard $(SRCDIR)/*.c)
HDRS   = $(wildcard $(INCDIR)/*.h)

TARGET = program_arm

# ---- 编译选项 ------------------------------------------------
INCS   = -I$(INCDIR)
LDLIBS = -lpthread -lrt          # 线程池用 pthread, mqueue 在 glibc 里

DEBUG ?= 1
ifeq ($(DEBUG),1)
  DBGFLAGS = -g
else
  DBGFLAGS =
endif

# 板子 rootfs 分区仅 7.0M, OTA 安装时 program_arm / .bak / .new 三份需同时在分区内。
# 因此默认 strip: 产物约 742K; 不 strip 为 1240K, 会导致下次 OTA 失败并可能写坏 jffs2。
# 需要保留符号表做调试时显式关闭: make STRIP=0
STRIP ?= 1

# -D_GNU_SOURCE: pthread_setname_np() 等 GNU 扩展需要它才会被声明,
#               否则会报 implicit declaration(参数类型对不上, 属于 UB)
#
# BUILD_STAMP: 每次 make 都重新求值的时间戳, 通过 -D 注入二进制, 供"程序版本"打印。
#   比源码里的 __DATE__/__TIME__ 可靠: __DATE__ 只在 main.c 被重新编译时才更新,
#   只改其它文件时它会停留在旧日期, 导致版本日期失真。
BUILD_STAMP := $(shell date '+%Y-%m-%d %H:%M:%S')

CFLAGS ?= -O2 $(DBGFLAGS) $(INCS) -D_GNU_SOURCE -DBUILD_STAMP='"$(BUILD_STAMP)"'

# 板子上不保证有匹配的动态库, 必须静态链接
LDFLAGS = -static

# ---- 目标 ----------------------------------------------------
.PHONY: all clean help

all: $(TARGET)

# 依赖里必须带上 HDRS: 否则只改 include/*.h 时 make 会判定产物已是最新而跳过编译,
# OTA 上去的其实是旧程序。加了这层依赖后不必再手工敲 -B。
$(TARGET): $(SRCS) $(HDRS)
	@echo "  CC ARM  $@  ($(words $(SRCS)) 个源文件)"
	@$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $(SRCS) $(LDLIBS)
ifeq ($(STRIP),1)
	@echo "  STRIP   $@"
	@$(CROSS)strip $@
endif

clean:
	@rm -f $(TARGET)
	@rm -f $(SRCDIR)/*.o
	@echo "  clean   已清除"

help:
	@echo "make            编 ARM 版  ($(TARGET))  ← 静态链接, 给板子"
	@echo "make DEBUG=0    不带调试信息"
	@echo "make STRIP=1    编完 strip 符号表"
	@echo "make clean      清除产物"
	@echo ""
	@echo "工具链: $(CC)"
