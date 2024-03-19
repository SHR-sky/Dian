#include <stdio.h>
#include "WiFi.h"


static char *TAG = "wifi station";

SemaphoreHandle_t sem;

// 状态机事件处理：static void event_handler(void * arg, esp_event_base_t event_base,
//                                           int32_t event_id, void * event_data)
static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if ((event_base == WIFI_EVENT) && (event_id == WIFI_EVENT_STA_START ||
                                       event_id == WIFI_EVENT_STA_DISCONNECTED))
    {
        ESP_LOGI(TAG, "begin to connect the AP");
        esp_wifi_connect();
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        xSemaphoreGive(sem);
    }
}

void WiFi_connect(void)
{
    sem = xSemaphoreCreateBinary();

    // nvs初始化：nvs_flash_init()
    ESP_ERROR_CHECK(nvs_flash_init());

    // 事件循环初始化:sp_event_loop_create_default();
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // 事件处理函数注册
    //  esp_err_t esp_event_handler_register(esp_event_base_t event_base, int32_t event_id,
    //                                       esp_event_handler_t event_handler, void * event_handler_arg)
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, event_handler, NULL));

    // 初始化阶段：esp_netif_init()；sp_netif_create_default_wifi_sta();esp_wifi_init()；
    ESP_ERROR_CHECK(esp_netif_init());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));

    // 配置阶段:esp_wifi_set_mode();esp_wifi_set_config();
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    wifi_config_t cfg = {
        .sta = {
            //用户名和密码根据实际情况修改
            .ssid = "荣耀Magic4 Pro",
            .password = "123321123",
        }};
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &cfg));

    // 启动阶段:esp_wifi_start()
    ESP_ERROR_CHECK(esp_wifi_start());

    while (1)
    {
        if (xSemaphoreTake(sem, portMAX_DELAY) == pdPASS)
        {
            ESP_LOGI(TAG, "connected to ap!");
        }
    }
}