#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "dist_vfs.h"

#ifdef __linux__
#include <unistd.h>
#else
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
// === 新增：ESP32 连网必须包含的头文件 ===
#include "esp_wifi.h"
#include "esp_event.h"
#endif

static const char *TAG = "SATE_APP";

void run_edfs_logic(uint8_t node_i, uint8_t node_j)
{
    init_node_identity(node_i, node_j); 
    init_dist_storage_system();
    init_ssp_network();                 

#ifdef __linux__
    usleep(500000); 
#else
    vTaskDelay(pdMS_TO_TICKS(500));
#endif

    ESP_LOGI(TAG, "测试 A: 正在写入本地文件...");
    FILE *f1 = fopen("/dist/local/test_a.txt", "w");
    if (f1)
    {
        for(int t=0;t<100;t++)
        fprintf(f1, "Hello Satellite Storage from (%d, %d) [HW Platform: %s]\n", node_i, node_j, 
#ifdef __linux__
                "Linux"
#else
                "ESP32"
#endif
                );
        fclose(f1);
        ESP_LOGI(TAG, "写入成功！");
    }

    ESP_LOGI(TAG, "测试 B: 正在尝试回环读取...");
    char loopback_path[64];
    sprintf(loopback_path, "/dist/remote/%d_%d/test_a.txt", node_i, node_j);
    FILE *f2 = fopen(loopback_path, "r");
    if (f2)
    {
        char buf[128];
        if (fgets(buf, sizeof(buf), f2)) {
            printf("[%s] 读取到的内容: %s", TAG, buf);
        }
        fclose(f2);
    }

#ifdef __linux__
    // 只有 Linux 的 (1, 1) 会主动发起跨硬件网络请求
    if (node_i == 1 && node_j == 1) {
        uint8_t target_j = 3; 
        ESP_LOGI("APP", "\n\n*** 测试 C: 尝试跨越中间节点，访问远端 ESP32 卫星 (%d, %d) ***", node_i, target_j);
        char remote_path[64];
        sprintf(remote_path, "/dist/remote/%d_%d/test_a.txt", node_i, target_j);
        
        // 我们等 3 秒钟，确保其他节点都已经完全启动
        usleep(3000000);
        
        // ================= 需要修改的地方：真正读取并打印 =================
        ESP_LOGI("APP", "准备调用 fopen，预期将发生阻塞等待远端回传...");
        FILE *f3 = fopen(remote_path, "r");
        
        if (f3) {
            ESP_LOGI("APP", "fopen 阻塞结束，已拿到有效文件指针！准备读取内容：");
            char buf[256] = {0};
            size_t bytes = fread(buf, 1, sizeof(buf) - 1, f3);
            if (bytes > 0) {
                printf("\n======================================================\n");
                printf("!!! 成功跨节点 (通过中继) 读取到远端文件 !!!\n");
                printf("读取到的字节数: %zu\n", bytes);
                printf("远端文件内容:\n%s\n", buf);
                printf("======================================================\n\n");
            } else {
                ESP_LOGE("APP", "文件已打开，但读取内容为空");
            }
            fclose(f3);
        } else {
            ESP_LOGE("APP", "跨节点读取失败，fopen 返回 NULL (可能超时或远端不存在)");
        }
        // ================= 增加远端写入测试 =================
        ESP_LOGI("APP", "\n\n*** 测试 D: 尝试跨越中间节点，向远端 ESP32 卫星写入文件 ***");
        char write_path[64];
        sprintf(write_path, "/dist/remote/%d_%d/upload_test.txt", node_i, target_j);
        
        FILE *f_write = fopen(write_path, "w");
        if (f_write) {
            for(int t=0; t<50; t++) {
                fprintf(f_write, "这是 Linux(1,1) 主动传给 ESP32(1,3) 的机密数据，第 %d 行\n", t);
            }
            ESP_LOGI("APP", "数据循环 fwrite 完成，准备 fclose 触发网络刷盘...");
            fclose(f_write); // 这里会真正触发 UDP 切片发送
            ESP_LOGI("APP", "网络刷盘结束！");
        }
        // ====================================================
        
    } else {
        ESP_LOGI("APP", "*** 当前 Linux 节点 (%d, %d) 进入静默监听状态，充当中继路由... ***\n", node_i, node_j);
    }
#else
    ESP_LOGI("APP", "*** [ESP32 硬件节点] 启动完毕，进入静默监听并随时准备响应！ ***\n");
#endif

    fflush(stdout);
}

#ifndef __linux__
// ESP32 极简 Wi-Fi 连接配置
#define WIFI_SSID "三楼"     // <--- 改为真实的 Wi-Fi 名称
#define WIFI_PASS "ww85890903" // <--- 改为真实的 Wi-Fi 密码

static void wifi_init_sta(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());
    
    ESP_LOGI(TAG, "====== 正在连接 Wi-Fi，请留意下方打印的【分配 IP】 ======");
    vTaskDelay(pdMS_TO_TICKS(5000)); // 给它 5 秒钟连网
}

void edfs_test_task(void *pvParameters)
{
    // ESP32 的身份固定为卫星 (1, 3)
    run_edfs_logic(1, 3);
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_flash_init();
    }
    // 先连接 Wi-Fi，再启动业务逻辑
    wifi_init_sta();
    xTaskCreate(edfs_test_task, "edfs_task", 4096, NULL, 5, NULL);
}
#endif

#ifdef __linux__
int main(int argc, char *argv[])
{
    uint8_t my_i = 1;
    uint8_t my_j = 1;

    if (argc >= 3) {
        my_i = atoi(argv[1]);
        my_j = atoi(argv[2]);
    }

    printf("=======================================\n");
    printf("启动卫星节点: 轨道面 %d, 序号 %d\n", my_i, my_j);
    printf("=======================================\n");

    run_edfs_logic(my_i, my_j);

    while (1) {
        sleep(1); 
    }
    return 0;
}
#endif