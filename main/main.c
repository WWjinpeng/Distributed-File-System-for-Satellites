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
#endif

static const char *TAG = "SATE_APP";

// 把坐标作为参数传进来
void run_edfs_logic(uint8_t node_i, uint8_t node_j)
{
    init_node_identity(node_i, node_j); // 初始化身份
    init_dist_storage_system();
    init_ssp_network();                 // 启动网络监听任务！

    // 稍微延时一下，等监听线程彻底跑起来
    #ifdef __linux__
    usleep(500000); // 0.5s
    #else
    vTaskDelay(pdMS_TO_TICKS(500));
    #endif

    // 场景 A: 本地写入
    ESP_LOGI(TAG, "测试 A: 正在写入本地文件...");
    FILE *f1 = fopen("/dist/local/test_a.txt", "w");
    if (f1)
    {
        fprintf(f1, "Hello Local Satellite Storage from (%d, %d)\n", node_i, node_j);
        fclose(f1);
        ESP_LOGI(TAG, "写入成功！");
    }

    // 场景 B: 回环读取验证
    ESP_LOGI(TAG, "测试 B: 正在尝试回环读取...");
    char loopback_path[64];
    sprintf(loopback_path, "/dist/remote/%d_%d/test_a.txt", node_i, node_j);
    FILE *f2 = fopen(loopback_path, "r");
    if (f2)
    {
        char buf[64];
        if (fgets(buf, sizeof(buf), f2)) {
            printf("[%s] 读取到的内容: %s", TAG, buf);
        }
        fclose(f2);
    }

    // 场景 C: 路由中继极限测试
    if (node_i == 1 && node_j == 1) {
        // 只有 1_1 主动去请求 1_3
        uint8_t target_j = 3; 
        ESP_LOGI("APP", "\n\n*** 测试 C: 尝试跨越中间节点，访问远端卫星 (%d, %d) ***", node_i, target_j);
        char remote_path[64];
        sprintf(remote_path, "/dist/remote/%d_%d/test_a.txt", node_i, target_j);
        fopen(remote_path, "r");
    } else {
        ESP_LOGI("APP", "*** 当前节点进入静默监听状态，随时准备充当网络中继路由... ***\n");
    }

    fflush(stdout);
}

#ifndef __linux__
void edfs_test_task(void *pvParameters)
{
    // ESP32 端目前写死为卫星 (1, 1)
    run_edfs_logic(1, 1);
    while (1)
    {
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
    xTaskCreate(edfs_test_task, "edfs_task", 4096, NULL, 5, NULL);
}
#endif

#ifdef __linux__
int main(int argc, char *argv[])
{
    uint8_t my_i = 1;
    uint8_t my_j = 1;

    // 允许通过命令行参数动态指定这颗卫星的坐标： ./sate_dfs_sim 1 2
    if (argc >= 3) {
        my_i = atoi(argv[1]);
        my_j = atoi(argv[2]);
    }

    printf("=======================================\n");
    printf("启动卫星节点: 轨道面 %d, 序号 %d\n", my_i, my_j);
    printf("=======================================\n");

    run_edfs_logic(my_i, my_j);

    // 关键：死循环保持进程不退出，否则后台的 UDP 监听线程会跟着死亡
    while (1) {
        sleep(1); 
    }
    return 0;
}
#endif