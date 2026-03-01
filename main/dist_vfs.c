#define _GNU_SOURCE
#include <dirent.h>   // 目录遍历支持
#include <sys/stat.h> // 获取文件大小支持
#include "dist_vfs.h"
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <ctype.h>
#include <stdlib.h>
#ifdef __linux__
// --- Linux 网络头文件 ---
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <time.h>      //Linux 超时机制支持
#else
// --- ESP32 网络及文件系统头文件 ---
#include "esp_vfs.h"
#include "esp_littlefs.h"
#include "lwip/sockets.h" // ESP32 的套接字库，和 Linux API 一样
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#endif

// ==========================================
// ====== 同步阻塞相关的全局状态与信号量 ======
// ==========================================

// 记录当前正在等待的远程文件状态
static char g_remote_cache_path[256] = {0};                // 期望保存的本地缓存路径

// --- 新增：远端写入控制变量 ---
static volatile int g_is_remote_writing = 0; // 是否正处于远端写入的缓存阶段
static uint8_t g_write_target_i = 0;         // 写入目标节点 i
static uint8_t g_write_target_j = 0;         // 写入目标节点 j
static char g_write_remote_path[256] = {0};  // 目标节点的逻辑路径
static char g_write_cache_path[256] = {0};   // 本地生成的临时写入缓存路径

static ikcpcb *g_kcp_client = NULL; // 供发起方 (例如拉取或写入数据) 使用的 KCP 实例
static ikcpcb *g_kcp_server = NULL; // 供接收方 (例如被动回传数据) 使用的 KCP 实例

// 记录当前 KCP 正在通信的对端坐标，方便 output 回调函数知道往哪发
static uint8_t g_kcp_target_i = 0;
static uint8_t g_kcp_target_j = 0;

// KCP 内存分配钩子映射
void* kcp_malloc(size_t size) { return malloc(size); }
void kcp_free(void* ptr) { free(ptr); }

// ====== [新增] Server 状态机 ======
#define SVR_IDLE 0
#define SVR_SENDING 1
#define SVR_RECEIVING 2

static volatile int g_svr_state = SVR_IDLE;
static FILE* g_svr_file = NULL;
static uint32_t g_svr_file_size = 0;
static uint32_t g_svr_processed = 0;

// ====== 废弃旧变量 ======
// 注意：之前定义的 g_sync_sem, g_ack_sem, g_remote_is_waiting, g_remote_received_bytes 等 
// 以及处理超时的代码统统不需要了！KCP 内部已经包含了这些！


#ifdef __linux__
static FILE *g_write_linux_file = NULL; // 记录当前打开的 Linux 远端写入文件指针
#else
static int g_write_esp_fd = -1; // 记录当前打开的 ESP32 远端写入文件描述符
#endif

static const char *TAG = "EDFS_VFS";
static node_coord_t g_local_node = {1, 1};

#define PC_LAN_IP "192.168.7.17"

#define ESP_LAN_IP "192.168.7.18"

// 简易路由表：判定下一跳应该走哪个物理设备的 IP
const char *get_node_ip(uint8_t j)
{
    if (j == 3)
    {
        return ESP_LAN_IP; // 设定 (1,3) 是 ESP32 硬件板
    }
    return PC_LAN_IP; // (1,1) 和 (1,2) 都在电脑仿真端
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
    if (strncmp(path, "/remote/", 8) == 0)
    {
        int target_i, target_j;
        if (sscanf(path + 8, "%d_%d/", &target_i, &target_j) == 2)
        {
            if (target_i == g_local_node.i && target_j == g_local_node.j)
            {
                const char *filename = strchr(path + 8, '/');
                if (filename)
                {
#ifdef __linux__
                    snprintf(out_path, max_len, "./sim_storage_%d_%d%s", g_local_node.i, g_local_node.j, filename);
#else
                    snprintf(out_path, max_len, "%s%s", MOUNT_POINT_PHYSICAL, filename);
#endif
                    ESP_LOGI(TAG, "检测到远程路径指向本地，已自动重定向");
                    return 0; // 本地处理
                }
            }
            else
            {
                snprintf(out_path, max_len, "SSP_FORWARD:%d_%d", target_i, target_j);
                return 1; // 远程转发
            }
        }
    }

    strncpy(out_path, path, max_len);
    return 0;
}

// 打印 SSP 报文
void ssp_debug_print_frame(const ssp_frame_t *frame)
{
    printf("\n--- [SSP FRAME DEBUG] ---\n");
    printf("Header:  0x%02X | Type: %02X\n", frame->start_byte, frame->type);
    printf("Route:   (%d, %d) -> (%d, %d)\n", frame->src_i, frame->src_j, frame->dst_i, frame->dst_j);
    printf("Payload Length: %d\n", frame->data_len);
    printf("Raw HEX: ");
    uint8_t *raw = (uint8_t *)frame;
    // 基础头 7 字节 (start1+src2+dst2+type1+len2) + payload长度
    for (size_t i = 0; i < (8 + frame->data_len); i++) 
    {
        printf("%02X ", raw[i]);
    }
    printf("\n-------------------------\n");
}



// ==========================================================
// ====== 【跨平台通用发送函数】 ======
// ==========================================================
void ssp_udp_send_packet(const ssp_frame_t *frame)
{
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0)
    {
        perror("Socket creation failed");
        return;
    }

    // 路由算法：计算下一跳坐标
    uint8_t next_i = g_local_node.i;
    uint8_t next_j = g_local_node.j;

    if (frame->dst_i > g_local_node.i)
        next_i++;
    else if (frame->dst_i < g_local_node.i)
        next_i--;
    else if (frame->dst_j > g_local_node.j)
        next_j++;
    else if (frame->dst_j < g_local_node.j)
        next_j--;

    struct sockaddr_in dest_addr;
    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(8000 + next_j);

    // 自动判断并选择下一跳设备的真实 IP
    const char *target_ip = get_node_ip(next_j);
    dest_addr.sin_addr.s_addr = inet_addr(target_ip);

    // [修改这里]
    int send_len = 8 + frame->data_len;
    sendto(sock, frame, send_len, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));

    printf("[%s] [网络层] 数据包: 终点(%d,%d) -> 下一跳(%d,%d) [IP:%s Port:%d, Len:%d]\n",
           TAG, frame->dst_i, frame->dst_j, next_i, next_j, target_ip, 8000 + next_j, send_len);

    close(sock);
}

// ==========================================================
// ====== [新增] KCP 底层发送回调函数 ======
// ==========================================================
// 当 KCP 需要发送数据时，它会调用这个函数。我们需要把 KCP 给的数据套上 SSP 路由头，用 UDP 发出去。
static int ssp_kcp_output_callback(const char *buf, int len, ikcpcb *kcp, void *user) {
    if (len > 1400) {
        ESP_LOGE(TAG, "[KCP] 致命错误：KCP 抛出的数据长度 (%d) 超过了 Payload 限制！", len);
        return -1;
    }

    ssp_frame_t route_frame;
    memset(&route_frame, 0, sizeof(ssp_frame_t));
    route_frame.start_byte = SSP_START_BYTE;
    route_frame.src_i = g_local_node.i;
    route_frame.src_j = g_local_node.j;
    route_frame.dst_i = g_kcp_target_i; // 使用全局记录的当前通信目标
    route_frame.dst_j = g_kcp_target_j;
    route_frame.type = SSP_TYPE_KCP_DATA; // 标记这是包裹了 KCP 数据的包
    route_frame.data_len = (uint16_t)len; // KCP 数据的长度

    // 将 KCP 吐出来的数据装进包心菜的最内层
    memcpy(route_frame.payload, buf, len); 

    // 直接复用我们之前写好的 UDP 发送函数
    ssp_udp_send_packet(&route_frame);
    return 0;
}

// ==========================================================
// ====== [新增] KCP 时钟驱动与状态更新线程 ======
// ==========================================================
// 获取当前系统毫秒时间戳的辅助函数
static IUINT32 iclock() {
    #ifdef __linux__
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (ts.tv_sec * 1000) + (ts.tv_nsec / 1000000);
    #else
    return (IUINT32)(esp_timer_get_time() / 1000);
    #endif
}

// 这个线程每 10ms 运行一次，驱动 KCP 检查是否需要重传或者发送积压的 ACK
#ifdef __linux__
static void* kcp_update_thread(void* arg)
#else
static void kcp_update_task(void* arg)
#endif
{
    while (1) {
        IUINT32 current_time = iclock();
        //先用局部变量接住指针，防止主线程随时置空
        ikcpcb *client = g_kcp_client;
        ikcpcb *server = g_kcp_server;
        if (client) ikcp_update(client, current_time); // [修改] 这里用局部的 client
        if (server) ikcp_update(server, current_time); // [修改] 这里用局部的 server
        
        // [新增] 驱动 Server 端后台发送大文件
        if (g_svr_state == SVR_SENDING && g_svr_file) {
            // 只要队列未满，就不停塞数据，KCP会负责打包和拥塞控制！
            while (ikcp_waitsnd(g_kcp_server) < 16 && g_svr_processed < g_svr_file_size) {
                char chunk[1024];
                size_t bytes = fread(chunk, 1, sizeof(chunk), g_svr_file);
                if (bytes > 0) {
                    ikcp_send(g_kcp_server, chunk, bytes);
                    g_svr_processed += bytes;
                } else break;
            }
            // 发完了，回归安静
            if (g_svr_processed >= g_svr_file_size) {
                fclose(g_svr_file); g_svr_file = NULL;
                g_svr_state = SVR_IDLE;
                ESP_LOGI(TAG, "=> [Server] 响应数据推送至 KCP 队列完毕！");
            }
        }

        #ifdef __linux__
        struct timespec req = {0, 10000000}; // 10ms
        nanosleep(&req, NULL);
        #else
        vTaskDelay(pdMS_TO_TICKS(10));
        #endif
    }
    #ifdef __linux__
    return NULL;
    #endif
}


// ==========================================================
// ====== 【KCP 改造】Client 读取远端文件 ======
// ==========================================================
static int fetch_remote_file_sync(int target_i, int target_j, const char* remote_path, char* out_cache_path) {
    g_kcp_target_i = target_i; g_kcp_target_j = target_j;
if (!g_kcp_client) {
        g_kcp_client = ikcp_create(0x1122, NULL); 
        g_kcp_client->output = ssp_kcp_output_callback;
        ikcp_nodelay(g_kcp_client, 1, 10, 2, 1);  // 开启 Turbo 模式
        ikcp_wndsize(g_kcp_client, 32, 32);      // ESP32 内存保护限制
    }  

    #ifdef __linux__
    snprintf(out_cache_path, 256, "./sim_storage_%d_%d/.cache_remote", g_local_node.i, g_local_node.j);
    #else
    snprintf(out_cache_path, 256, "%s/.cache_remote", MOUNT_POINT_PHYSICAL);
    #endif

    ESP_LOGI(TAG, "=> [KCP流] 发起拉取: %s", remote_path);
    char req[256]; req[0] = 0x01; // CMD_READ
    strcpy(req + 1, remote_path);
    ikcp_send(g_kcp_client, req, 2 + strlen(remote_path)); // 发送请求

    FILE *f = fopen(out_cache_path, "wb");
    if (!f) return -1;

    uint32_t total_size = 0, received = 0;
    int header_parsed = 0;
    IUINT32 start_time = iclock();

    // 不断地从 KCP 管道里“吸”数据，丢包重传 KCP 在后台自动搞定
    while (1) {
        if (iclock() - start_time > 5000) { ESP_LOGE(TAG, "KCP 连接超时！"); break; }
        char buf[1024];
        int len = ikcp_recv(g_kcp_client, buf, sizeof(buf));
        if (len > 0) {
            start_time = iclock(); // 喂狗
            int data_offset = 0;
            if (!header_parsed) {
                memcpy(&total_size, buf, 4); // 报文头4字节是总大小
                data_offset = 4; header_parsed = 1;
                ESP_LOGI(TAG, "=> [KCP流] 文件总大小: %u", total_size);
                if (total_size == 0) break;
            }
            if (len - data_offset > 0) {
                fwrite(buf + data_offset, 1, len - data_offset, f);
                received += (len - data_offset);
            }
            if (header_parsed && received >= total_size) {
                ESP_LOGI(TAG, "=> [KCP流] 下载 100%% 成功！"); break;
            }
        }
        #ifdef __linux__
        usleep(10000); 
        #else
        vTaskDelay(pdMS_TO_TICKS(10));
        #endif
    }
    fclose(f);
    return (received >= total_size && header_parsed) ? 0 : -1;
}

// ==========================================================
// ====== 【KCP 改造】Client 延迟刷盘 (写入) ======
// ==========================================================
static void flush_remote_write_sync(void) {
    FILE *f = (fopen)(g_write_cache_path, "rb"); if (!f) return;
    fseek(f, 0, SEEK_END); uint32_t total_size = ftell(f); fseek(f, 0, SEEK_SET);

    g_kcp_target_i = g_write_target_i; g_kcp_target_j = g_write_target_j;
    if (!g_kcp_client) {
        g_kcp_client = ikcp_create(0x1122, NULL);
        g_kcp_client->output = ssp_kcp_output_callback;
        ikcp_nodelay(g_kcp_client, 1, 10, 2, 1);
        ikcp_wndsize(g_kcp_client, 32, 32);
    }

    ESP_LOGI(TAG, "=> [KCP流] 开启高速刷盘, 大小: %u", total_size);
    char req[256]; req[0] = 0x02; // CMD_WRITE
    memcpy(req + 1, &total_size, 4);
    strcpy(req + 5, g_write_remote_path);
    ikcp_send(g_kcp_client, req, 6 + strlen(g_write_remote_path));

    uint32_t sent = 0;
    while (sent < total_size) {
        // 如果发送队列没满，就疯狂往里塞
        if (ikcp_waitsnd(g_kcp_client) < 16) {
            char chunk[1024];
            size_t bytes = fread(chunk, 1, sizeof(chunk), f);
            ikcp_send(g_kcp_client, chunk, bytes);
            sent += bytes;
        } else {
            #ifdef __linux__
            usleep(10000);
            #else
            vTaskDelay(pdMS_TO_TICKS(10));
            #endif
        }
    }
    (fclose)(f);

    ESP_LOGI(TAG, "=> 数据已全推入队列，等待目标节点落盘 ACK...");
    IUINT32 start_time = iclock();
    while (1) {
        if (iclock() - start_time > 5000) { ESP_LOGE(TAG, "KCP 等待 ACK 超时"); break; }
        char buf[16];
        if (ikcp_recv(g_kcp_client, buf, sizeof(buf)) > 0) {
            if (buf[0] == 0x04) { ESP_LOGI(TAG, "=> [KCP流] 远端落盘 100%% 完成！"); break; }
        }
        #ifdef __linux__
        usleep(10000);
        #else
        vTaskDelay(pdMS_TO_TICKS(10));
        #endif
    }
    //ikcp_release(g_kcp_client); g_kcp_client = NULL;
}

// ==========================================================
// ====== 【KCP 改造】Client 拉取快照 ======
// ==========================================================
int edfs_pull_snapshot(uint8_t target_i, uint8_t target_j) {
    g_kcp_target_i = target_i; g_kcp_target_j = target_j;
    if (!g_kcp_client) {
        g_kcp_client = ikcp_create(0x1122, NULL); 
        g_kcp_client->output = ssp_kcp_output_callback;
        ikcp_nodelay(g_kcp_client, 1, 10, 2, 1);
        ikcp_wndsize(g_kcp_client, 32, 32);
    }

    #ifdef __linux__
    snprintf(g_remote_cache_path, 256, "./sim_storage_%d_%d/.cache_snapshot", g_local_node.i, g_local_node.j);
    #else
    snprintf(g_remote_cache_path, 256, "%s/.cache_snapshot", MOUNT_POINT_PHYSICAL);
    #endif

    ESP_LOGI(TAG, "=> 正在向卫星 (%d, %d) 发送快照拉取请求...", target_i, target_j);
    char req[2] = {0x03, 0x00}; // CMD_LIST_DIR
    ikcp_send(g_kcp_client, req, 2);

    FILE *f = fopen(g_remote_cache_path, "wb"); if (!f) return -1;
    uint32_t total_size = 0, received = 0;
    int header_parsed = 0; IUINT32 start_time = iclock();

    while (1) {
        if (iclock() - start_time > 5000) { break; }
        char buf[1024]; int len = ikcp_recv(g_kcp_client, buf, sizeof(buf));
        if (len > 0) {
            start_time = iclock(); 
            int data_offset = 0;
            if (!header_parsed) { memcpy(&total_size, buf, 4); data_offset = 4; header_parsed = 1; if (total_size == 0) break; }
            if (len - data_offset > 0) { fwrite(buf + data_offset, 1, len - data_offset, f); received += (len - data_offset); }
            if (header_parsed && received >= total_size) break;
        }
        #ifdef __linux__
        usleep(10000); 
        #else
        vTaskDelay(pdMS_TO_TICKS(10));
        #endif
    }
    fclose(f);

    f = fopen(g_remote_cache_path, "r");
    if (f) {
        printf("\n"); char pbuf[256];
        while (fgets(pbuf, sizeof(pbuf), f)) printf("\033[1;36m%s\033[0m", pbuf);
        printf("\n"); fclose(f);
    }
    return 0;
}


#ifndef __linux__
// ESP32 VFS 绑定
static int vfs_dist_open(void *ctx, const char *path, int flags, int mode)
{
    char target[256];
    int is_remote = resolve_path(path, target, sizeof(target));

    if (is_remote)
    {
        int di, dj;
        if (sscanf(target, "SSP_FORWARD:%d_%d", &di, &dj) == 2)
        {
            char cache_path[256];
            ESP_LOGI(TAG, "[VFS] 拦截到 ESP32 远程读取请求 %s", path);

            if (fetch_remote_file_sync(di, dj, path, cache_path) == 0)
            {
                return open(cache_path, flags, mode);
            }
            else
            {
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
FILE *dist_linux_fopen(const char *path, const char *mode)
{
    char target[256];
    int is_remote = 0;

    if (strncmp(path, "/dist", 5) == 0)
    {
        is_remote = resolve_path(path + 5, target, sizeof(target));
    }
    else
    {
        strcpy(target, path);
    }

    if (is_remote)
    {
        int di, dj;
        if (sscanf(target, "SSP_FORWARD:%d_%d", &di, &dj) == 2)
        {
            // 判定是否是写模式 (w, a, r+, w+ 等)
            int is_write_mode = (strchr(mode, 'w') != NULL) || (strchr(mode, 'a') != NULL) || (strchr(mode, '+') != NULL);

            if (is_write_mode)
            {
                // ============== 【写入模式拦截】 ==============
                snprintf(g_write_cache_path, sizeof(g_write_cache_path), "./sim_storage_%d_%d/.cache_write_remote", g_local_node.i, g_local_node.j);
                FILE *f = fopen(g_write_cache_path, mode);
                if (f)
                {
                    g_is_remote_writing = 1;
                    g_write_target_i = di;
                    g_write_target_j = dj;
                    strcpy(g_write_remote_path, path);
                    g_write_linux_file = f;
                    ESP_LOGI(TAG, "[VFS] 拦截到远程写入请求，创建本地缓存: %s", g_write_cache_path);
                }
                return f; // 将本地缓存文件指针骗过上层返回
            }
            else
            {
                // ============== 【原有的读取模式拦截】 ==============
                char cache_path[256];
                ESP_LOGI(TAG, "[VFS] 拦截到远程读取请求 %s，目标节点(%d, %d)", path, di, dj);
                if (fetch_remote_file_sync(di, dj, path, cache_path) == 0)
                {
                    return fopen(cache_path, mode);
                }
                else
                {
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
int dist_linux_fclose(FILE *stream)
{
    if (!stream)
        return 0;

    // 提前判断并清理状态，防止递归
    int should_flush = 0;
    if (g_is_remote_writing && stream == g_write_linux_file)
    {
        should_flush = 1;
        g_is_remote_writing = 0;   // <--- 提前置零
        g_write_linux_file = NULL; // <--- 提前清空
    }

    int ret = fclose(stream); // 执行真实的关闭

    // 如果刚才判断需要刷盘，现在安全地触发
    if (should_flush)
    {
        ESP_LOGI(TAG, "[VFS] 远端缓存文件关闭，正在触发延迟刷盘(网络发送)...");
        flush_remote_write_sync();
    }

    return ret;
}
int dist_linux_fseek(FILE *stream, long offset, int whence) { return fseek(stream, offset, whence); }
#endif

// ==========================================================
// ====== 【LIST_DIR：同 READ 的自动重传机制】 ======
// ==========================================================


esp_err_t init_dist_storage_system(void)
{


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
    if (ret != ESP_OK)
        return ret;

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
static void *ssp_rx_thread(void *arg)
#else
static void ssp_rx_task(void *arg)
#endif
{
    int rx_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (rx_sock < 0)
    {
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

    if (bind(rx_sock, (struct sockaddr *)&rx_addr, sizeof(rx_addr)) < 0)
    {
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

    while (1)
    {
        int len = recvfrom(rx_sock, rx_buffer, sizeof(rx_buffer), 0, (struct sockaddr *)&source_addr, &socklen);
        if (len > 0)
        {
            ssp_frame_t *frame = (ssp_frame_t *)rx_buffer;

            if (frame->start_byte == SSP_START_BYTE) {
                // 中继路由：一秒识别，毫秒转发
                if (frame->dst_i != g_local_node.i || frame->dst_j != g_local_node.j) {
                    #ifdef __linux__
                    usleep(5000); // KCP 中继可降至 5ms
                    #else
                    vTaskDelay(pdMS_TO_TICKS(5));
                    #endif
                    ssp_udp_send_packet(frame);
                    continue; 
                }

                // 本地解包：识别 KCP 数据包
                if (frame->type == SSP_TYPE_KCP_DATA) {
                    // 如果是 Client 收到了回复
                    if (g_kcp_client && frame->src_i == g_kcp_target_i && frame->src_j == g_kcp_target_j) {
                        ikcp_input(g_kcp_client, frame->payload, frame->data_len);
                    } else {
                        // 我们是 Server，收到了别人的请求
                        if (!g_kcp_server) {
                            g_kcp_server = ikcp_create(0x1122, NULL);
                            g_kcp_server->output = ssp_kcp_output_callback;
                            ikcp_nodelay(g_kcp_server, 1, 10, 2, 1);
                            ikcp_wndsize(g_kcp_server, 32, 32);
                        }
                        g_kcp_target_i = frame->src_i; 
                        g_kcp_target_j = frame->src_j;
                        ikcp_input(g_kcp_server, frame->payload, frame->data_len);

                        // 提取命令，处理业务逻辑
                        char buf[1024]; int len;
                        while ((len = ikcp_recv(g_kcp_server, buf, sizeof(buf))) > 0) {
                            if (g_svr_state == SVR_IDLE) {
                                uint8_t cmd = buf[0];
                                if (cmd == 0x01) { // 收到 READ 请求
                                    char filename[256]; strcpy(filename, buf + 1);
                                    const char *fbase = strrchr(filename, '/'); if (!fbase) fbase = filename; else fbase++; 
                                    char local_path[512];
                                    #ifdef __linux__
                                    snprintf(local_path, sizeof(local_path), "./sim_storage_%d_%d/%s", g_local_node.i, g_local_node.j, fbase);
                                    #else
                                    snprintf(local_path, sizeof(local_path), "%s/%s", MOUNT_POINT_PHYSICAL, fbase);
                                    #endif
                                    g_svr_file = fopen(local_path, "r");
                                    if (g_svr_file) {
                                        fseek(g_svr_file, 0, SEEK_END); g_svr_file_size = ftell(g_svr_file); fseek(g_svr_file, 0, SEEK_SET);
                                        ikcp_send(g_kcp_server, (const char*)&g_svr_file_size, 4); // 先发大小头
                                        g_svr_processed = 0; g_svr_state = SVR_SENDING;
                                    } else {
                                        uint32_t zero = 0; ikcp_send(g_kcp_server, (const char*)&zero, 4);
                                    }
                                } 
                                else if (cmd == 0x02) { // 收到 WRITE 请求
                                    memcpy(&g_svr_file_size, buf + 1, 4);
                                    char filename[256]; strcpy(filename, buf + 5);
                                    const char *fbase = strrchr(filename, '/'); if (!fbase) fbase = filename; else fbase++; 
                                    char local_path[512];
                                    #ifdef __linux__
                                    snprintf(local_path, sizeof(local_path), "./sim_storage_%d_%d/%s", g_local_node.i, g_local_node.j, fbase);
                                    #else
                                    snprintf(local_path, sizeof(local_path), "%s/%s", MOUNT_POINT_PHYSICAL, fbase);
                                    #endif
                                    g_svr_file = fopen(local_path, "wb");
                                    g_svr_processed = 0; g_svr_state = SVR_RECEIVING;
                                    ESP_LOGI(TAG, "=> [Server] 开始接收写入, 预期 %u 字节", g_svr_file_size);
                                }
                                else if (cmd == 0x03) { // 收到 LIST_DIR 请求
                                    char base_path[256], snap_path[256];
                                    #ifdef __linux__
                                    snprintf(base_path, 256, "./sim_storage_%d_%d", g_local_node.i, g_local_node.j);
                                    snprintf(snap_path, 256, "./sim_storage_%d_%d/.snapshot.txt", g_local_node.i, g_local_node.j);
                                    #else
                                    snprintf(base_path, 256, "%s", MOUNT_POINT_PHYSICAL);
                                    snprintf(snap_path, 256, "%s/.snapshot.txt", MOUNT_POINT_PHYSICAL);
                                    #endif
                                    FILE *sf = fopen(snap_path, "w");
                                    if (sf) {
                                        DIR *dir = opendir(base_path);
                                        if (dir) {
                                            struct dirent *ent;
                                            fprintf(sf, "=================================================\n Node (%d, %d) Storage Snapshot\n=================================================\n", g_local_node.i, g_local_node.j);
                                            while ((ent = readdir(dir)) != NULL) {
                                                if (ent->d_name[0] == '.') continue;
                                                char filepath[512]; snprintf(filepath, sizeof(filepath), "%s/%s", base_path, ent->d_name);
                                                struct stat st;
                                                if (stat(filepath, &st) == 0) fprintf(sf, "[FILE] %-20s | Size: %ld bytes\n", ent->d_name, (long)st.st_size);
                                            }
                                            closedir(dir);
                                        }
                                        fclose(sf);
                                    }
                                    g_svr_file = fopen(snap_path, "r");
                                    if (g_svr_file) {
                                        fseek(g_svr_file, 0, SEEK_END); g_svr_file_size = ftell(g_svr_file); fseek(g_svr_file, 0, SEEK_SET);
                                        ikcp_send(g_kcp_server, (const char*)&g_svr_file_size, 4);
                                        g_svr_processed = 0; g_svr_state = SVR_SENDING;
                                        ESP_LOGI(TAG, "=> [Server] 开始推送快照数据，大小: %u 字节", g_svr_file_size); // [加上这行日志]
                                    }
                                }
                            } else if (g_svr_state == SVR_RECEIVING) { // 持续收流模式
                                fwrite(buf, 1, len, g_svr_file);
                                g_svr_processed += len;
                                if (g_svr_processed >= g_svr_file_size) {
                                    fclose(g_svr_file); g_svr_file = NULL;
                                    g_svr_state = SVR_IDLE;
                                    char ack = 0x04; ikcp_send(g_kcp_server, &ack, 1);
                                    ESP_LOGI(TAG, "=> [Server] 文件接收落盘完成，已发送 ACK！");
                                }
                            }
                        }
                    }
                }
            }
            else
            {
                ESP_LOGE(TAG, "收到未知格式数据包，已丢弃");
            }
        }
    }

#ifdef __linux__
    return NULL;
#endif
}

esp_err_t init_ssp_network(void) {
    // 绑定 KCP 内存分配钩子
    ikcp_allocator(kcp_malloc, kcp_free);

#ifdef __linux__
    pthread_t tid_rx, tid_kcp;
    if (pthread_create(&tid_rx, NULL, ssp_rx_thread, NULL) != 0) {
        ESP_LOGE(TAG, "创建 Linux 接收线程失败");
        return ESP_FAIL;
    }
    pthread_detach(tid_rx); 
    
    // [新增] 启动 KCP 驱动线程
    if (pthread_create(&tid_kcp, NULL, kcp_update_thread, NULL) != 0) {
        ESP_LOGE(TAG, "创建 KCP 时钟线程失败");
        return ESP_FAIL;
    }
    pthread_detach(tid_kcp);
#else
    xTaskCreate(ssp_rx_task, "ssp_rx_task", 8192, NULL, 5, NULL);
    // [新增] 启动 KCP 驱动任务
    xTaskCreate(kcp_update_task, "kcp_update", 8192, NULL, 5, NULL);
#endif
    return ESP_OK;
}