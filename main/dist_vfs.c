#include "dist_vfs.h"
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <ctype.h>

#ifdef __linux__
// --- Linux 网络头文件 ---
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/socket.h> 
#include <netinet/in.h> 
#include <arpa/inet.h>  
#else
// --- ESP32 网络及文件系统头文件 ---
#include "esp_vfs.h"
#include "esp_littlefs.h"
#include "lwip/sockets.h" // ESP32 的套接字库，和 Linux API 一样
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
    for(size_t i = 0; i < (11 + frame->path_len); i++) {
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

    int send_len = 12 + frame->path_len + frame->data_len;
    sendto(sock, frame, send_len, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    
    printf("[%s] [网络层] 数据包: 终点(%d,%d) -> 下一跳(%d,%d) [IP:%s Port:%d, Len:%d]\n", 
           TAG, frame->dst_i, frame->dst_j, next_i, next_j, target_ip, 8000 + next_j, send_len);

    close(sock);
}


#ifndef __linux__
// ESP32 VFS 绑定
static int vfs_dist_open(void *ctx, const char *path, int flags, int mode) {
    char target[256];
    resolve_path(path, target, sizeof(target));
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
            ssp_frame_t frame;
            ssp_pack_request(&frame, SSP_TYPE_READ, (uint8_t)di, (uint8_t)dj, path);
            printf("[%s] [远程请求] 拦截到远程路径，生成 SSP 报文:\n", TAG);
            ssp_debug_print_frame(&frame);
            ssp_udp_send_packet(&frame); // 发送到下一跳
        }
        return NULL; 
    }
    
    printf("[%s] [本地路由] %s -> %s\n", TAG, path, target);
    return fopen(target, mode);
}

size_t dist_linux_fread(void *ptr, size_t size, size_t nmemb, FILE *stream) { return fread(ptr, size, nmemb, stream); }
size_t dist_linux_fwrite(const void *ptr, size_t size, size_t nmemb, FILE *stream) { return fwrite(ptr, size, nmemb, stream); }
int dist_linux_fclose(FILE *stream) { if (!stream) return 0; return fclose(stream); }
int dist_linux_fseek(FILE *stream, long offset, int whence) { return fseek(stream, offset, whence); }
#endif


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

                    ssp_frame_t resp_frame;
                    memset(&resp_frame, 0, sizeof(ssp_frame_t));
                    resp_frame.start_byte = SSP_START_BYTE;
                    resp_frame.src_i = g_local_node.i;
                    resp_frame.src_j = g_local_node.j;
                    resp_frame.dst_i = frame->src_i; 
                    resp_frame.dst_j = frame->src_j;
                    resp_frame.type = SSP_TYPE_RESP_DATA; 

                    FILE *f = fopen(local_path, "r");
                    if (f) {
                        size_t bytes = fread(resp_frame.payload, 1, sizeof(resp_frame.payload) - 1, f);
                        resp_frame.data_len = bytes;
                        resp_frame.path_len = 0; 
                        fclose(f);
                        ESP_LOGI(TAG, "=> 成功读取本地文件，准备回传 %zu 字节数据", bytes);
                    } else {
                        resp_frame.data_len = 0;
                        strcpy(resp_frame.payload, "FILE_NOT_FOUND");
                        ESP_LOGE(TAG, "=> 请求的文件不存在，返回错误信息");
                    }

                    // ESP32 现在也可以正大光明地回传文件了！
                    ssp_udp_send_packet(&resp_frame);
                }
                // ============== 处理【数据响应】 ==============
                else if (frame->type == SSP_TYPE_RESP_DATA) {
                    ESP_LOGI(TAG, "\n================ [ 远端文件内容到达 ] ================");
                    if (frame->data_len > 0) {
                        printf("来自卫星 (%d, %d) 的文件数据:\n%s\n", frame->src_i, frame->src_j, frame->payload);
                    } else {
                        printf("来自卫星 (%d, %d) 报错: %s\n", frame->src_i, frame->src_j, frame->payload);
                    }
                    printf("======================================================\n");
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