#include <stdio.h>
#include <string.h>


#define IDF_mode 0

float Yaw,Pitch,Roll;

#if (IDF_mode == -1)

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>

#include <esp_event.h>
#include <nvs_flash.h>
#include <esp_netif.h>

#include <WiFi.h>
#include <lwip/sockets.h>
#include <driver/gpio.h>

static char *TAG = "tcp_client";

static int sock;

// 功能二
#define LED_PIN GPIO_NUM_2 // led对应的gpio口
void led_init()
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1 << LED_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg)); // 对gpio进行初始化配置
}
void led_on()
{
    ESP_ERROR_CHECK(gpio_set_level(LED_PIN, 1));
}
void led_off()
{
    ESP_ERROR_CHECK(gpio_set_level(LED_PIN, 0));
}
static void my_led_task(void *pvParameters) // 控制led亮灭的任务
{
    led_init();
    while (1)
    {
        char cmd;
        int count = recv(sock, &cmd, 1, 0); // 接收服务端发来的控制指令
        if (count > 0)
        {
            if (cmd == '1')
            {
                led_on(); // led灯亮
            }
            else if (cmd == '0')
            {
                led_off(); // led灯灭
            }
        }
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // esp32连接wifi热点
    WiFi_connect();

    // 创建socket:socket();
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0)
    {
        ESP_LOGE(TAG, "create socket failed!");
        return;
    }
    ESP_LOGI(TAG, "create socket successfully!");

    // 初始化server的地址结构体sockaddr_in
    struct sockaddr_in destaddr = {};
    destaddr.sin_family = AF_INET;
    destaddr.sin_port = htons(3333);                      // 填写网络调试助手服务端实际端口
    destaddr.sin_addr.s_addr = inet_addr("10.26.65.239"); // 填写网络调试助手服务端实际IP地址

    // 建立socket连接：
    socklen_t len = sizeof(struct sockaddr);
    if (connect(sock, (struct sockaddr *)&destaddr, len) < 0)
    {
        ESP_LOGE(TAG, "connect to server failed!");
        close(sock);
        return;
    }
    ESP_LOGI(TAG, "connect to server successfully!");

    // 创建任务来实现功能一和功能二同时运行
    xTaskCreate(my_led_task, "my_led", 4096, NULL, 5, NULL);

    // 功能一
    while (1)
    {
        // 发送数据给服务端：send();
        char buff[512] = "hello, I am tcp_client!";
        send(sock, buff, strlen(buff), 0);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}


#endif


#if (IDF_mode == 0)

#include "LED.h"
#include "FreeRTOS/freertos.h"
#include "FreeRTOS/task.h"

void app_main(void)
{
    while(1)
    {
        led_init();
        led_on();
        vTaskDelay(1000);
        led_off();
        vTaskDelay(1000);
    }
    
}

#endif

#if (IDF_mode == 1)

#include "myusart.h"
void app_main()
{
    myusart_init();
    myusart_test();
}

#endif



#if (IDF_mode == 2)

#include "myusart.h"
#include "myiic.h"

void app_main()
{
    char message[1000]="";
    float uint[6];

    myusart_init();
    myiic_Init();

    int16_t ax,ay,az,gx,gy,gz;
    mpu6050_get_six(&gx, &gy, &gz, &ax, &ay, &az);
    change_unit(&gx, &gy, &gz, &ax, &ay, &az, uint);
    sprintf(message,"gx:%f, gy:%f, gz:%f, ax:%f, ay:%f, az:%f \n",uint[0],uint[1],uint[2],uint[3],uint[4],uint[5]); 
    myusart_print(message);
    mpu6050_get_data();
    sprintf(message,"yaw:%d, roll:%d, pitch:%d \n",Yaw,Roll,Pitch);
    myusart_print(message);
}


#endif