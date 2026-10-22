#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include "sd.h"
// 本文件属于哪个分模块打印开关(必须在 #include "log.h" 之前定义)
#define LOG_MODULE  LOG_MOD_SYS
#include "log.h"

#define SD_MOUNT_POINT "/mnt/sdcard"
#define SD_DEVICE      "/dev/mmcblk0p1"

// 检查是否已挂载（改进版）
int sd_is_mounted(void) {
    // 1. 先检查设备是否存在
    if (access(SD_DEVICE, F_OK) != 0) {
        return 0;
    }

    // 2. 检查挂载记录
    FILE *fp = fopen("/proc/mounts", "r");
    if (fp == NULL) return 0;

    char line[256];
    int found = 0;
    while (fgets(line, sizeof(line), fp) != NULL) {
        if (strstr(line, SD_MOUNT_POINT) != NULL && 
            strstr(line, SD_DEVICE) != NULL) {
            found = 1;
            break;
        }
    }
    fclose(fp);
    
    // 3. 验证挂载点目录是否存在且可访问
    if (found) {
        struct stat st;
        if (stat(SD_MOUNT_POINT, &st) != 0) {
            return 0;
        }
        return S_ISDIR(st.st_mode) ? 1 : 0;
    }
    
    return found;
}

// 挂载SD卡（改进版）
int sd_mount(void) {
    printf("[SD] ▶ 挂载开始: %s -> %s\n", SD_DEVICE, SD_MOUNT_POINT);

    // 检查设备是否存在
    if (access(SD_DEVICE, F_OK) != 0) {
        printf("[SD] ❌ 挂载失败: 设备 %s 不存在: %s\n", SD_DEVICE, strerror(errno));
        return -1;
    }

    // 清理无效挂载
    printf("[SD]   清理旧的挂载记录\n");
    system("umount " SD_MOUNT_POINT " 2>/dev/null");
    system("umount -l " SD_MOUNT_POINT " 2>/dev/null");
    system("umount -f " SD_MOUNT_POINT " 2>/dev/null");

    // 创建挂载点
    if (mkdir(SD_MOUNT_POINT, 0755) != 0) {
        if (errno != EEXIST) {
            printf("[SD] ❌ 挂载失败: 创建挂载点 %s 出错: %s\n",
                   SD_MOUNT_POINT, strerror(errno));
            return -1;
        }
        printf("[SD]   挂载点已存在: %s\n", SD_MOUNT_POINT);
    } else {
        printf("[SD]   挂载点已创建: %s\n", SD_MOUNT_POINT);
    }

    // 挂载
    if (mount(SD_DEVICE, SD_MOUNT_POINT, "vfat", MS_NOATIME, NULL) != 0) {
        printf("[SD] ❌ 挂载失败: mount(%s -> %s) 出错: %s\n",
               SD_DEVICE, SD_MOUNT_POINT, strerror(errno));
        return -1;
    }

    // 验证挂载是否真正成功
    FILE *test = fopen(SD_MOUNT_POINT "/.test", "w");
    if (test == NULL) {
        printf("[SD] ⚠️ 挂载成功但不可写, 已回滚卸载: %s\n", strerror(errno));
        umount(SD_MOUNT_POINT);
        return -1;
    }
    fclose(test);
    remove(SD_MOUNT_POINT "/.test");

    printf("[SD] ✅ 挂载完成: %s -> %s (可读写)\n", SD_DEVICE, SD_MOUNT_POINT);
    return 0;
}

// 卸载SD卡
int sd_umount(void) {
    if (!sd_is_mounted()) {
        printf("[SD] ⚠️ 卸载跳过: 当前未挂载\n");
        return 0;
    }

    printf("[SD] ▶ 卸载开始: %s\n", SD_MOUNT_POINT);

    chdir("/");
    sync();
    sleep(1);

    if (umount(SD_MOUNT_POINT) != 0) {
        printf("[SD] ❌ 普通卸载失败: %s, 尝试强制卸载\n", strerror(errno));
        system("umount -l " SD_MOUNT_POINT " 2>/dev/null");
        system("umount -f " SD_MOUNT_POINT " 2>/dev/null");
        printf("[SD] ⚠️ 已发出强制卸载命令, 请用 mount 命令确认结果\n");
        return -1;
    }
    printf("[SD] ✅ 卸载完成: %s\n", SD_MOUNT_POINT);
    return 0;
}

// 拼接完整路径
static int build_path(const char *filename, char *buf, int buf_size) {
    if (filename == NULL || filename[0] == '\0') {
        filename = "ceshi.txt";
    }
    int n = snprintf(buf, (size_t)buf_size, "%s/%s", SD_MOUNT_POINT, filename);
    if (n < 0 || n >= buf_size) {
        printf("[SD] ❌ 路径拼接失败: 文件名过长 %s (缓冲区 %d 字节不够)\n",
               filename, buf_size);
        return -1;
    }
    return 0;
}

// 覆盖写入
int sd_write_cover(const char *filename, const char *data) {
    if (data == NULL) {
        printf("[SD] ❌ 覆盖写入失败: data 为 NULL\n");
        return -1;
    }

    if (!sd_is_mounted()) {
        printf("[SD] ❌ 覆盖写入失败: SD 卡未挂载\n");
        return -1;
    }

    char file_path[256];
    if (build_path(filename, file_path, sizeof(file_path)) != 0) return -1;

    FILE *fp = fopen(file_path, "w");
    if (fp == NULL) {
        printf("[SD] ❌ 覆盖写入失败: 打开 %s 出错: %s\n", file_path, strerror(errno));
        return -1;
    }

    size_t len = strlen(data);
    if (len > 0) {
        size_t written = fwrite(data, 1, len, fp);
        if (written != len) {
            printf("[SD] ❌ 覆盖写入失败: 实际写入 %zu/%zu 字节\n", written, len);
            fclose(fp);
            return -1;
        }
    }

    fclose(fp);
    sync();
    printf("[SD] ✅ 覆盖写入完成: %s (%zu 字节)\n", file_path, len);
    return 0;
}

// 追加写入
int sd_write_append(const char *filename, const char *data) {
    if (data == NULL) {
        printf("[SD] ❌ 追加写入失败: data 为 NULL\n");
        return -1;
    }

    if (!sd_is_mounted()) {
        printf("[SD] ❌ 追加写入失败: SD 卡未挂载\n");
        return -1;
    }

    char file_path[256];
    if (build_path(filename, file_path, sizeof(file_path)) != 0) return -1;

    FILE *fp = fopen(file_path, "a");
    if (fp == NULL) {
        printf("[SD] ❌ 追加写入失败: 打开 %s 出错: %s\n", file_path, strerror(errno));
        return -1;
    }

    size_t len = strlen(data);
    if (len > 0) {
        size_t written = fwrite(data, 1, len, fp);
        if (written != len) {
            printf("[SD] ❌ 追加写入失败: 实际写入 %zu/%zu 字节\n", written, len);
            fclose(fp);
            return -1;
        }
    }

    fclose(fp);
    sync();
    printf("[SD] ✅ 追加写入完成: %s (%zu 字节)\n", file_path, len);
    return 0;
}

// 读取文件
int sd_read(const char *filename, char *buf, int buf_size) {
    if (buf == NULL || buf_size <= 0) {
        printf("[SD] ❌ 读取失败: 参数无效(buf=%p, size=%d)\n", (void *)buf, buf_size);
        return -1;
    }

    if (!sd_is_mounted()) {
        printf("[SD] ❌ 读取失败: SD 卡未挂载\n");
        return -1;
    }

    buf[0] = '\0';

    char file_path[256];
    if (build_path(filename, file_path, sizeof(file_path)) != 0) return -1;

    FILE *fp = fopen(file_path, "r");
    if (fp == NULL) {
        printf("[SD] ❌ 读取失败: 打开 %s 出错: %s\n", file_path, strerror(errno));
        return -1;
    }

    size_t total = 0;
    size_t n;
    while (total < (size_t)buf_size - 1 &&
           (n = fread(buf + total, 1, (size_t)buf_size - 1 - total, fp)) > 0) {
        total += n;
    }

    buf[total] = '\0';
    fclose(fp);

    if (total == 0) {
        printf("[SD] ⚠️ 文件为空: %s\n", file_path);
    } else {
        printf("[SD] ✅ 读取完成: %s (%zu 字节)\n", file_path, total);
    }
    return (int)total;
}

// 删除文件
int sd_remove(const char *filename) {
    if (!sd_is_mounted()) {
        printf("[SD] ❌ 删除失败: SD 卡未挂载\n");
        return -1;
    }

    char file_path[256];
    if (build_path(filename, file_path, sizeof(file_path)) != 0) return -1;

    if (remove(file_path) != 0) {
        printf("[SD] ❌ 删除失败: %s: %s\n", file_path, strerror(errno));
        return -1;
    }
    printf("[SD] ✅ 删除完成: %s\n", file_path);
    return 0;
}