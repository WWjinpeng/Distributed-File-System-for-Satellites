// main/dist_vfs.h
#ifndef DIST_VFS_H
#define DIST_VFS_H

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#ifdef __linux__
    #include <pthread.h> // Linux 多线程支持
    typedef int esp_err_t;
    #define ESP_OK 0
    #define ESP_FAIL -1
    #define ESP_LOGI(tag, fmt, ...) printf("[%s] " fmt "\n", tag, ##__VA_ARGS__)
    #define ESP_LOGE(tag, fmt, ...) printf("[%s] ERROR: " fmt "\n", tag, ##__VA_ARGS__)
    
    #define fopen   dist_linux_fopen
    #define fread   dist_linux_fread
    #define fwrite  dist_linux_fwrite
    #define fclose  dist_linux_fclose
    #define fseek   dist_linux_fseek

    FILE* dist_linux_fopen(const char *path, const char *mode);
    size_t  dist_linux_fread(void *ptr, size_t size, size_t nmemb, FILE *stream);
    size_t  dist_linux_fwrite(const void *ptr, size_t size, size_t nmemb, FILE *stream);
    int     dist_linux_fclose(FILE *stream);
    int     dist_linux_fseek(FILE *stream, long offset, int whence);
#else
    #include "lwip/sockets.h"
    #include "esp_err.h"
    #include "esp_log.h"
#endif

// 挂载点
#define MOUNT_POINT_PHYSICAL "/local"
#define MOUNT_POINT_LOGICAL "/dist"

// --- SSP 协议定义 ---
#define SSP_START_BYTE 0x5A
#define SSP_TYPE_READ      0x01
#define SSP_TYPE_WRITE     0x02
#define SSP_TYPE_LIST_DIR  0x03 // [新增] 目录快照拉取请求
#define SSP_TYPE_ACK       0x04 // [新增] 接收确认包
#define SSP_TYPE_RESP_DATA 0x82 // [新增] 数据响应包类型

#pragma pack(push, 1)
typedef struct
{
    uint8_t start_byte; //帧起始标志
    uint8_t src_i, src_j;
    uint8_t dst_i, dst_j;
    uint8_t type; //报文类型
    uint16_t path_len; //文件路径的字符串长度
    uint32_t data_len; //实际数据部分的长度
    uint32_t file_offset; // 当前分片在文件中的偏移量
    uint32_t file_size;   // 文件的总大小
    char payload[512]; 
} ssp_frame_t;
#pragma pack(pop)

typedef struct
{
    uint8_t i;
    uint8_t j;
} node_coord_t;

// 接口
esp_err_t init_dist_storage_system(void);
void init_node_identity(uint8_t i, uint8_t j);
esp_err_t init_ssp_network(void);
// [新增] 供应用层调用的拉取快照函数
int edfs_pull_snapshot(uint8_t target_i, uint8_t target_j);
#endif