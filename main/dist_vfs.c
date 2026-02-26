#define _GNU_SOURCE
#include "dist_vfs.h"
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <ctype.h>

#ifdef __linux__
// --- Linux 网络头文件 ---
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/socket.h> 
#include <netinet/in.h> 
#include <arpa/inet.h>  
#include <semaphore.h>   //Linux 信号量支持
#include <time.h>        //Linux 超时机制支持
#else
// --- ESP32 网络及文件系统头文件 ---
#include "esp_vfs.h"
#include "esp_littlefs.h"
#include "lwip/sockets.h" // ESP32 的套接字库，和 Linux API 一样
#include "freertos/FreeRTOS.h" 
#include "freertos/semphr.h"   //ESP32 信号量支持
#endif


// ==========================================
// ====== 同步阻塞相关的全局状态与信号量 ======
// ==========================================
#ifdef __linux__
static sem_t g_sync_sem;
#else
static SemaphoreHandle_t g_sync_sem = NULL;
#endif

// 记录当前正在等待的远程文件状态
static volatile int g_remote_is_waiting = 0;      // 是否正在等待
static char g_remote_wait_path[256] = {0};        // 正在等待的远端路径
static char g_remote_cache_path[256] = {0};       // 期望保存的本地缓存路径
static volatile int g_remote_fetch_status = -1;   // 获取结果: 0成功, -1失败
static volatile uint32_t g_remote_received_bytes = 0; // 已接收的字节数
static volatile uint32_t g_remote_total_bytes = 0;    // 预期接收的总字节数


// --- 新增：远端写入控制变量 ---
static volatile int g_is_remote_writing = 0;      // 是否正处于远端写入的缓存阶段
static uint8_t g_write_target_i = 0;              // 写入目标节点 i
static uint8_t g_write_target_j = 0;              // 写入目标节点 j
static char g_write_remote_path[256] = {0};       // 目标节点的逻辑路径
static char g_write_cache_path[256] = {0};        // 本地生成的临时写入缓存路径
#ifdef __linux__
static FILE* g_write_linux_file = NULL;           // 记录当前打开的 Linux 远端写入文件指针
#else
static int g_write_esp_fd = -1;                   // 记录当前打开的 ESP32 远端写入文件描述符
#endif

static const char *TAG = "EDFS_VFS";
static node_coord_t g_local_node = {1, 1};

#define PC_LAN_IP "192.168.7.17"  

#define ESP_LAN_IP "192.168.7.18" 

// 简易路由表：判定下一跳应该走哪个物理设备的 IP
const char* get_node_ip(uint8_t j) {
    if (j == 3) {
        return ESP_LAN_IP; // 设定 (1,3) 是 ESP32 硬件板
    }
    return PC_LAN_IP;      // (1,1) 和 (1,2) 都在电脑仿真端
}
// ==========================================================

// 初始化节点身份
void init_node_identity(uint8_t i, uint8_t j)
{
    g_local_node.i = i;
    g_local_node.j = j;
    ESP_LOGI(TAG, "节点身份已确认为: 卫星 (%d, %d)", i, j);
}

static int resolve_path(const char *path, char *out_path, size_t max_len)
{
    // 1. 本地逻辑路径: /local/...
    if (strncmp(path, "/local", 6) == 0)
    {
        #ifdef __linux__
            snprintf(out_path, max_len, "./sim_storage_%d_%d%s", g_local_node.i, g_local_node.j, path + 6);
        #else
            snprintf(out_path, max_len, "%s%s", MOUNT_POINT_PHYSICAL, path + 6);
        #endif
        return 0;
    }

    // 2. 远程逻辑路径处理: /remote/i_j/...
    if (strncmp(path, "/remote/", 8) == 0) {
        int target_i, target_j;
        if (sscanf(path + 8, "%d_%d/", &target_i, &target_j) == 2) {
            if (target_i == g_local_node.i && target_j == g_local_node.j) {
                const char *filename = strchr(path + 8, '/'); 
                if (filename) {
                    #ifdef __linux__
                        snprintf(out_path, max_len, "./sim_storage_%d_%d%s", g_local_node.i, g_local_node.j, filename);
                    #else
                        snprintf(out_path, max_len, "%s%s", MOUNT_POINT_PHYSICAL, filename);
                    #endif
                    ESP_LOGI(TAG, "检测到远程路径指向本地，已自动重定向");
                    return 0; // 本地处理
                }
            } else {
                snprintf(out_path, max_len, "SSP_FORWARD:%d_%d", target_i, target_j);
                return 1; // 远程转发
            }
        }
    }
    
    strncpy(out_path, path, max_len);
    return 0;
}

// 打印 SSP 报文
void ssp_debug_print_frame(const ssp_frame_t *frame) {
    printf("\n--- [SSP FRAME DEBUG] ---\n");
    printf("Header:  0x%02X | Type: %02X\n", frame->start_byte, frame->type);
    printf("Route:   (%d, %d) -> (%d, %d)\n", frame->src_i, frame->src_j, frame->dst_i, frame->dst_j);
    printf("Payload: %s\n", frame->payload);
    printf("Raw HEX: ");
    uint8_t *raw = (uint8_t *)frame;
    for(size_t i = 0; i < (19 + frame->path_len); i++) {
        printf("%02X ", raw[i]);
    }
    printf("\n-------------------------\n");
}

void ssp_pack_request(ssp_frame_t *frame, uint8_t type, uint8_t di, uint8_t dj, const char *path) {
    memset(frame, 0, sizeof(ssp_frame_t));
    frame->start_byte = SSP_START_BYTE;
    frame->src_i = g_local_node.i;
    frame->src_j = g_local_node.j;
    frame->dst_i = di;
    frame->dst_j = dj;
    frame->type = type;
    frame->path_len = (uint16_t)strlen(path);
    frame->data_len = 0; 
    strncpy(frame->payload, path, sizeof(frame->payload) - 1);
}

// ==========================================================
// ====== 【跨平台通用发送函数】 ======
// ==========================================================
void ssp_udp_send_packet(const ssp_frame_t *frame) {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("Socket creation failed");
        return;
    }

    // 路由算法：计算下一跳坐标
    uint8_t next_i = g_local_node.i;
    uint8_t next_j = g_local_node.j;

    if (frame->dst_i > g_local_node.i) next_i++; 
    else if (frame->dst_i < g_local_node.i) next_i--; 
    else if (frame->dst_j > g_local_node.j) next_j++; 
    else if (frame->dst_j < g_local_node.j) next_j--; 

    struct sockaddr_in dest_addr;
    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(8000 + next_j);
    
    // 自动判断并选择下一跳设备的真实 IP
    const char* target_ip = get_node_ip(next_j);
    dest_addr.sin_addr.s_addr = inet_addr(target_ip); 

    int send_len = 20 + frame->path_len + frame->data_len;
    sendto(sock, frame, send_len, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    
    printf("[%s] [网络层] 数据包: 终点(%d,%d) -> 下一跳(%d,%d) [IP:%s Port:%d, Len:%d]\n", 
           TAG, frame->dst_i, frame->dst_j, next_i, next_j, target_ip, 8000 + next_j, send_len);

    close(sock);
}


// ==========================================================
// ====== 【核心：同步阻塞获取远端文件】 ======
// ==========================================================
static int fetch_remote_file_sync(int target_i, int target_j, const char* remote_path, char* out_cache_path) {
    // 1. 设置全局等待状态 (简单起见，V1版本我们只允许同一时刻一个线程发起远端请求)
    g_remote_is_waiting = 1;
    g_remote_fetch_status = -1;
    strncpy(g_remote_wait_path, remote_path, sizeof(g_remote_wait_path) - 1);
    
    // 生成一个本地缓存文件的路径
#ifdef __linux__
    snprintf(g_remote_cache_path, sizeof(g_remote_cache_path), "./sim_storage_%d_%d/.cache_remote", g_local_node.i, g_local_node.j);
#else
    snprintf(g_remote_cache_path, sizeof(g_remote_cache_path), "%s/.cache_remote", MOUNT_POINT_PHYSICAL);
#endif
    strcpy(out_cache_path, g_remote_cache_path);

    // 2. 打包并发送读取请求
    ssp_frame_t frame;
    ssp_pack_request(&frame, SSP_TYPE_READ, (uint8_t)target_i, (uint8_t)target_j, remote_path);
    ESP_LOGI(TAG, "=> 发起同步远端读取请求，准备挂起当前线程等待数据...");
    ssp_udp_send_packet(&frame);

    // 3. 阻塞等待网络层唤醒（带有超时机制，防止死锁）
#ifdef __linux__
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += 10; // 设置 10 秒超时
    if (sem_timedwait(&g_sync_sem, &ts) != 0) {
        ESP_LOGE(TAG, "等待远端文件响应超时！");
    }
#else
    // ESP32: 等待 3000 毫秒 (3秒)
    if (xSemaphoreTake(g_sync_sem, pdMS_TO_TICKS(10000)) != pdTRUE) {
        ESP_LOGE(TAG, "等待远端文件响应超时！");
    }
#endif

    // 4. 被唤醒或超时后，清理状态并返回结果
    g_remote_is_waiting = 0;
    return g_remote_fetch_status; // 如果网络层成功写入缓存，此处为 0
}


// ==========================================================
// ====== 【新增核心：关闭文件时将缓存切片刷入远端】 ======
// ==========================================================
static void flush_remote_write_sync(void) {
    FILE *f = fopen(g_write_cache_path, "rb");
    if (!f) return;
    
    // 获取缓存文件总大小
    fseek(f, 0, SEEK_END);
    uint32_t total_size = ftell(f);
    fseek(f, 0, SEEK_SET);

    ESP_LOGI(TAG, "[VFS] 开始将本地写入缓存刷入远端节点(%d,%d)，总大小: %u 字节", 
             g_write_target_i, g_write_target_j, total_size);

    uint32_t current_offset = 0;
    uint16_t path_len = strlen(g_write_remote_path);

    // 计算 payload 中留给纯数据的最大空间
    size_t max_data_len = sizeof(((ssp_frame_t*)0)->payload) - path_len - 1;

    // 处理空文件的情况
    if (total_size == 0) {
        ssp_frame_t frame;
        memset(&frame, 0, sizeof(ssp_frame_t));
        frame.start_byte = SSP_START_BYTE;
        frame.src_i = g_local_node.i;
        frame.src_j = g_local_node.j;
        frame.dst_i = g_write_target_i;
        frame.dst_j = g_write_target_j;
        frame.type = SSP_TYPE_WRITE;
        frame.file_offset = 0;
        frame.file_size = 0;
        frame.path_len = path_len;
        frame.data_len = 0;
        strcpy(frame.payload, g_write_remote_path); // 放入路径
        ssp_udp_send_packet(&frame);
    }

    // 大文件分片循环发送
    while (current_offset < total_size) {
        ssp_frame_t frame;
        memset(&frame, 0, sizeof(ssp_frame_t));
        frame.start_byte = SSP_START_BYTE;
        frame.src_i = g_local_node.i;
        frame.src_j = g_local_node.j;
        frame.dst_i = g_write_target_i;
        frame.dst_j = g_write_target_j;
        frame.type = SSP_TYPE_WRITE;
        frame.file_offset = current_offset;
        frame.file_size = total_size;
        frame.path_len = path_len;

        // 1. 先把路径拷贝到 payload 头部
        strcpy(frame.payload, g_write_remote_path);

        // 2. 计算还能装多少数据，并读取到 payload 的路径之后
        size_t bytes_to_read = total_size - current_offset;
        if (bytes_to_read > max_data_len) bytes_to_read = max_data_len;
        
        size_t bytes = fread(frame.payload + path_len, 1, bytes_to_read, f);
        frame.data_len = bytes;
        
        ssp_udp_send_packet(&frame);
        current_offset += bytes;

        #ifdef __linux__
        struct timespec req = {0, 5000000}; // 5ms
        nanosleep(&req, NULL);
        #else
        vTaskDelay(pdMS_TO_TICKS(10)); // ESP32 延迟 10ms
        #endif
    }
    (fclose)(f);
    ESP_LOGI(TAG, "[VFS] 远端文件刷盘完成！");
}



#ifndef __linux__
// ESP32 VFS 绑定
static int vfs_dist_open(void *ctx, const char *path, int flags, int mode) {
    char target[256];
    int is_remote = resolve_path(path, target, sizeof(target));
    
    if (is_remote) {
        int di, dj;
        if (sscanf(target, "SSP_FORWARD:%d_%d", &di, &dj) == 2) {
            char cache_path[256];
            ESP_LOGI(TAG, "[VFS] 拦截到 ESP32 远程读取请求 %s", path);
            
            if (fetch_remote_file_sync(di, dj, path, cache_path) == 0) {
                return open(cache_path, flags, mode);
            } else {
                return -1; // ESP32 VFS 失败返回 -1
            }
        }
    }
    return open(target, flags, mode);
}
static ssize_t vfs_dist_read(void *ctx, int fd, void *dst, size_t size) { return read(fd, dst, size); }
static ssize_t vfs_dist_write(void *ctx, int fd, const void *data, size_t size) { return write(fd, data, size); }
static int vfs_dist_close(void *ctx, int fd) { return close(fd); }
static off_t vfs_dist_lseek(void *ctx, int fd, off_t offset, int mode) { return lseek(fd, offset, mode); }
#endif

#ifdef __linux__
#undef fopen   
#undef fread
#undef fwrite
#undef fclose
#undef fseek

// Linux 宏替换绑定
// 替换原有的 dist_linux_fopen
FILE* dist_linux_fopen(const char *path, const char *mode) {
    char target[256];
    int is_remote = 0;

    if (strncmp(path, "/dist", 5) == 0) {
        is_remote = resolve_path(path + 5, target, sizeof(target));
    } else {
        strcpy(target, path);
    }
    
    if (is_remote) {
        int di, dj;
        if (sscanf(target, "SSP_FORWARD:%d_%d", &di, &dj) == 2) {
            // 判定是否是写模式 (w, a, r+, w+ 等)
            int is_write_mode = (strchr(mode, 'w') != NULL) || (strchr(mode, 'a') != NULL) || (strchr(mode, '+') != NULL);
            
            if (is_write_mode) {
                // ============== 【写入模式拦截】 ==============
                snprintf(g_write_cache_path, sizeof(g_write_cache_path), "./sim_storage_%d_%d/.cache_write_remote", g_local_node.i, g_local_node.j);
                FILE *f = fopen(g_write_cache_path, mode);
                if (f) {
                    g_is_remote_writing = 1;
                    g_write_target_i = di;
                    g_write_target_j = dj;
                    strcpy(g_write_remote_path, path);
                    g_write_linux_file = f;
                    ESP_LOGI(TAG, "[VFS] 拦截到远程写入请求，创建本地缓存: %s", g_write_cache_path);
                }
                return f; // 将本地缓存文件指针骗过上层返回
            } else {
                // ============== 【原有的读取模式拦截】 ==============
                char cache_path[256];
                ESP_LOGI(TAG, "[VFS] 拦截到远程读取请求 %s，目标节点(%d, %d)", path, di, dj);
                if (fetch_remote_file_sync(di, dj, path, cache_path) == 0) {
                    return fopen(cache_path, mode);
                } else {
                    return NULL; 
                }
            }
        }
    }
    return fopen(target, mode);
}

size_t dist_linux_fread(void *ptr, size_t size, size_t nmemb, FILE *stream) { return fread(ptr, size, nmemb, stream); }
size_t dist_linux_fwrite(const void *ptr, size_t size, size_t nmemb, FILE *stream) { return fwrite(ptr, size, nmemb, stream); }
// 替换原有的 dist_linux_fclose
int dist_linux_fclose(FILE *stream) { 
    if (!stream) return 0; 
    
    // 提前判断并清理状态，防止递归
    int should_flush = 0;
    if (g_is_remote_writing && stream == g_write_linux_file) {
        should_flush = 1;
        g_is_remote_writing = 0;   // <--- 提前置零
        g_write_linux_file = NULL; // <--- 提前清空
    }

    int ret = fclose(stream); // 执行真实的关闭
    
    // 如果刚才判断需要刷盘，现在安全地触发
    if (should_flush) {
        ESP_LOGI(TAG, "[VFS] 远端缓存文件关闭，正在触发延迟刷盘(网络发送)...");
        flush_remote_write_sync();
    }
    
    return ret; 
}
int dist_linux_fseek(FILE *stream, long offset, int whence) { return fseek(stream, offset, whence); }
#endif


esp_err_t init_dist_storage_system(void)
{

// 初始化跨平台二值信号量
#ifdef __linux__
    sem_init(&g_sync_sem, 0, 0); // 初始值为0
#else
    g_sync_sem = xSemaphoreCreateBinary();
#endif

#ifdef __linux__
    char storage_dir[64];
    snprintf(storage_dir, sizeof(storage_dir), "./sim_storage_%d_%d", g_local_node.i, g_local_node.j);
    ESP_LOGI(TAG, "初始化 Linux 仿真环境，专属物理目录: %s", storage_dir);
    mkdir(storage_dir, 0775);
    return 0;
#else
    ESP_LOGI(TAG, "正在初始化 ESP32 硬件 Flash (LittleFS)");
    esp_vfs_littlefs_conf_t conf = {
        .base_path = MOUNT_POINT_PHYSICAL,
        .partition_label = "littlefs",
        .format_if_mount_failed = true};

    esp_err_t ret = esp_vfs_littlefs_register(&conf);
    if (ret != ESP_OK) return ret;

    esp_vfs_t dist_vfs = {
        .flags = ESP_VFS_FLAG_CONTEXT_PTR,
        .open_p = &vfs_dist_open,
        .read_p = &vfs_dist_read,
        .write_p = &vfs_dist_write,
        .close_p = &vfs_dist_close,
        .lseek_p = &vfs_dist_lseek,
    };
    ESP_LOGI(TAG, "正在注册逻辑挂载点: %s", MOUNT_POINT_LOGICAL);
    return esp_vfs_register(MOUNT_POINT_LOGICAL, &dist_vfs, NULL);
#endif
}

// ==========================================================
// --- UDP 网络接收与监听模块 ---
// ==========================================================

#ifndef __linux__
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

#ifdef __linux__
static void* ssp_rx_thread(void* arg)
#else
static void ssp_rx_task(void* arg)
#endif
{
    int rx_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (rx_sock < 0) {
        ESP_LOGE(TAG, "无法创建接收 Socket");
        #ifdef __linux__
        return NULL;
        #else
        vTaskDelete(NULL);
        #endif
    }

    struct sockaddr_in rx_addr;
    memset(&rx_addr, 0, sizeof(rx_addr));
    rx_addr.sin_family = AF_INET;
    rx_addr.sin_addr.s_addr = htonl(INADDR_ANY); 
    rx_addr.sin_port = htons(8000 + g_local_node.j);

    if (bind(rx_sock, (struct sockaddr *)&rx_addr, sizeof(rx_addr)) < 0) {
        ESP_LOGE(TAG, "Socket 绑定失败，端口: %d", 8000 + g_local_node.j);
        close(rx_sock);
        #ifdef __linux__
        return NULL;
        #else
        vTaskDelete(NULL);
        #endif
    }

    ESP_LOGI(TAG, "SSP 网络监听已启动，持续监听本地端口: %d", 8000 + g_local_node.j);

    char rx_buffer[1024];
    struct sockaddr_in source_addr;
    socklen_t socklen = sizeof(source_addr);

    while (1) {
        int len = recvfrom(rx_sock, rx_buffer, sizeof(rx_buffer), 0, (struct sockaddr *)&source_addr, &socklen);
        if (len > 0) {
            ssp_frame_t *frame = (ssp_frame_t *)rx_buffer;
            
            if (frame->start_byte == SSP_START_BYTE) {


                if (frame->path_len > 0 && frame->path_len < sizeof(frame->payload)) {
                    frame->payload[frame->path_len] = '\0';
                }
                if (frame->data_len > 0 && frame->data_len < sizeof(frame->payload)) {
                    frame->payload[frame->data_len] = '\0';
                }
                //包拦截与中继转发
                if (frame->dst_i != g_local_node.i || frame->dst_j != g_local_node.j) {
                    ESP_LOGI(TAG, "=> [中继路由] 收到来自 (%d,%d) 发往 (%d,%d) 的包！目标不是我，执行转发...", 
                             frame->src_i, frame->src_j, frame->dst_i, frame->dst_j);
                    
                    #ifdef __linux__
                    usleep(100000); // 0.1秒
                    #else
                    vTaskDelay(pdMS_TO_TICKS(100)); // ESP32 延时
                    #endif
                    
                    ssp_udp_send_packet(frame); // 通用发送，无需 ifdef限制
                    continue; 
                }
                
                // ============== 处理【读取请求】 ==============
                if (frame->type == SSP_TYPE_READ) {
                    ESP_LOGI(TAG, "=> 收到远端节点 (%d, %d) 的读取请求: %s", frame->src_i, frame->src_j, frame->payload);
                    
                    const char *filename = strrchr(frame->payload, '/');
                    if (!filename) filename = frame->payload;
                    else filename++; 

                    char local_path[2000];
                    #ifdef __linux__
                    snprintf(local_path, sizeof(local_path), "./sim_storage_%d_%d/%s", g_local_node.i, g_local_node.j, filename);
                    #else
                    snprintf(local_path, sizeof(local_path), "%s/%s", MOUNT_POINT_PHYSICAL, filename);
                    #endif

                    FILE *f = fopen(local_path, "r");
                    if (f) {
                        // 获取文件总大小
                        fseek(f, 0, SEEK_END);
                        uint32_t total_size = ftell(f);
                        fseek(f, 0, SEEK_SET);

                        ESP_LOGI(TAG, "=> 开始发送文件，总大小: %u 字节", total_size);

                        uint32_t current_offset = 0;
                        while (current_offset < total_size) {
                            ssp_frame_t resp_frame;
                            memset(&resp_frame, 0, sizeof(ssp_frame_t));
                            resp_frame.start_byte = SSP_START_BYTE;
                            resp_frame.src_i = g_local_node.i;
                            resp_frame.src_j = g_local_node.j;
                            resp_frame.dst_i = frame->src_i; 
                            resp_frame.dst_j = frame->src_j;
                            resp_frame.type = SSP_TYPE_RESP_DATA; 
                            
                            // 设置分片信息
                            resp_frame.file_offset = current_offset;
                            resp_frame.file_size = total_size;

                            // 计算本分片要读取的长度
                            size_t bytes_to_read = total_size - current_offset;
                            if (bytes_to_read > sizeof(resp_frame.payload) - 1) {
                                bytes_to_read = sizeof(resp_frame.payload) - 1;
                            }

                            size_t bytes = fread(resp_frame.payload, 1, bytes_to_read, f);
                            resp_frame.data_len = bytes;
                            resp_frame.path_len = 0; 
                            
                            ssp_udp_send_packet(&resp_frame);
                            current_offset += bytes;

                            // 关键：加一点微小的延时，防止连续发送导致 UDP 缓冲区溢出丢包
                            #ifdef __linux__
                            struct timespec req = {0, 5000000}; // 5ms
                            nanosleep(&req, NULL);
                            #else
                            vTaskDelay(pdMS_TO_TICKS(10)); // ESP32 延迟 10ms
                            #endif
                        }
                        fclose(f);
                        ESP_LOGI(TAG, "=> 文件全部分片发送完毕！");
                    } else {
                        // 文件不存在时的报错处理不变
                        ssp_frame_t err_frame;
                        memset(&err_frame, 0, sizeof(ssp_frame_t));
                        err_frame.start_byte = SSP_START_BYTE;
                        err_frame.src_i = g_local_node.i;
                        err_frame.src_j = g_local_node.j;
                        err_frame.dst_i = frame->src_i; 
                        err_frame.dst_j = frame->src_j;
                        err_frame.type = SSP_TYPE_RESP_DATA;
                        err_frame.data_len = 0;
                        strcpy(err_frame.payload, "FILE_NOT_FOUND");
                        ssp_udp_send_packet(&err_frame);
                    }
                }
                else if(frame->type == SSP_TYPE_WRITE){
                    // 1. 从 payload 头部提取路径
                    char req_path[256] = {0};
                    if (frame->path_len > 0 && frame->path_len < sizeof(req_path)) {
                        strncpy(req_path, frame->payload, frame->path_len);
                    }
                    
                    const char *filename = strrchr(req_path, '/');
                    if (!filename) filename = req_path;
                    else filename++; 

                    char local_path[2000];
                    #ifdef __linux__
                    snprintf(local_path, sizeof(local_path), "./sim_storage_%d_%d/%s", g_local_node.i, g_local_node.j, filename);
                    #else
                    snprintf(local_path, sizeof(local_path), "%s/%s", MOUNT_POINT_PHYSICAL, filename);
                    #endif

                    // 2. 计算纯数据在 payload 中的起始指针
                    char *data_ptr = frame->payload + frame->path_len;

                    // 3. 落盘逻辑
                    FILE *f;
                    if (frame->file_offset == 0) {
                        f = fopen(local_path, "wb"); // 第一片：覆盖重建
                    } else {
                        f = fopen(local_path, "r+b"); // 后续片：修改
                    }

                    if (f) {
                        fseek(f, frame->file_offset, SEEK_SET);
                        fwrite(data_ptr, 1, frame->data_len, f);
                        fclose(f);
                        
                        ESP_LOGI(TAG, "=> 接收到远端写入分片: %s [进度 %u / %u]", 
                                 filename, frame->file_offset + frame->data_len, frame->file_size);
                        
                        if (frame->file_offset + frame->data_len >= frame->file_size) {
                            ESP_LOGI(TAG, "=> 远端文件 [%s] 完整写入落盘完毕！", filename);
                        }
                    } else {
                        ESP_LOGE(TAG, "无法打开本地文件供远端写入: %s", local_path);
                    }
                }
                // ============== 处理【数据响应】 ==============
                else if (frame->type == SSP_TYPE_RESP_DATA) {
                    if (frame->data_len > 0) {
                        if (g_remote_is_waiting) {
                            FILE *cache_f;
                            // 偏移量为 0 说明是第一个包，用 "w" 模式覆盖旧文件并重置计数器
                            if (frame->file_offset == 0) {
                                cache_f = fopen(g_remote_cache_path, "wb");
                                g_remote_received_bytes = 0;
                                g_remote_total_bytes = frame->file_size;
                            } else {
                                // 后续的包使用 "r+" 模式（读写模式），以便使用 fseek 定位
                                cache_f = fopen(g_remote_cache_path, "r+b"); 
                            }
                            
                            if (cache_f) {
                                // 根据报文里的偏移量写入数据，这种方式天然抵抗 UDP 乱序
                                fseek(cache_f, frame->file_offset, SEEK_SET);
                                fwrite(frame->payload, 1, frame->data_len, cache_f);
                                fclose(cache_f);
                                
                                g_remote_received_bytes += frame->data_len;
                                ESP_LOGI(TAG, "已接收分片: 偏移量 %u，当前进度: %u / %u 字节", 
                                         frame->file_offset, g_remote_received_bytes, g_remote_total_bytes);
                                
                                // 判断是否全部接收完毕
                                if (g_remote_received_bytes >= g_remote_total_bytes) {
                                    g_remote_fetch_status = 0; 
                                    ESP_LOGI(TAG, "====== 文件所有分片重组完毕！正在唤醒应用层... ======");
                                    
                                    #ifdef __linux__
                                    sem_post(&g_sync_sem);
                                    #else
                                    xSemaphoreGive(g_sync_sem);
                                    #endif
                                }
                            } else {
                                ESP_LOGE(TAG, "无法打开本地缓存文件进行重组！");
                            }
                        }
                    } else {
                        ESP_LOGE(TAG, "来自卫星 (%d, %d) 报错: %s", frame->src_i, frame->src_j, frame->payload);
                        if (g_remote_is_waiting) {
                            g_remote_fetch_status = -1; // 失败
                            #ifdef __linux__
                            sem_post(&g_sync_sem);
                            #else
                            xSemaphoreGive(g_sync_sem);
                            #endif
                        }
                    }
                }

            } else {
                ESP_LOGE(TAG, "收到未知格式数据包，已丢弃");
            }
        }
    }
    
    #ifdef __linux__
    return NULL;
    #endif
}

esp_err_t init_ssp_network(void) {
#ifdef __linux__
    pthread_t tid;
    if (pthread_create(&tid, NULL, ssp_rx_thread, NULL) != 0) {
        ESP_LOGE(TAG, "创建 Linux 接收线程失败");
        return ESP_FAIL;
    }
    pthread_detach(tid); 
#else
    xTaskCreate(ssp_rx_task, "ssp_rx_task", 8192, NULL, 5, NULL);
#endif
    return ESP_OK;
}