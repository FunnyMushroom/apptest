#ifndef SD_H
#define SD_H

// 挂载/卸载
int sd_mount(void);
int sd_umount(void);

// 文件操作
int sd_write_cover(const char *filename, const char *data);
int sd_write_append(const char *filename, const char *data);
int sd_read(const char *filename, char *buf, int buf_size);
int sd_remove(const char *filename);

#endif